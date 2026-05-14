#include "hwgc.h"

#define TYPE_PCI_HWGC_DEVICE "hwgc"

typedef struct HWGCState HWGCState;

DECLARE_INSTANCE_CHECKER(HWGCState, HWGC, TYPE_PCI_HWGC_DEVICE)

struct HWGCState
{
    PCIDevice pdev;
    MemoryRegion mmio;

    // 工作线程相关
    QemuThread thread;
    QemuMutex thr_mutex;
    QemuCond thr_cond;

    uint32_t status; // soft write to enable device
    uint32_t irq_status;
    uint32_t state;
    uint32_t sub_state;
    uint32_t wake_state;
    uint32_t wake_sub_state;

    // data
    bool access_ok;
    int par_allocate_iml_sel, par_allocate_sel, is_full_value, wait_num;
    uintptr_t alloc_top, alloc_end, result;
    size_t alloc_available, want_to_allocate, actual_word_size;

    // pars
    struct HWGC_PARALLOCATE_PARS pars;

    // irq pars and res
    struct HWGC_IRQ_PARS irq_pars;

    CPUState *cpu;
    HWGCTLBEntry tlb_cache[HWGC_TLB_SIZE];
};

// PCI/PCIe 支持两种中断机制
//      MSI: 边沿出发 不需要维持状态 发一次会自动无效掉
//      Legacy INTx: 电平触发 需要手动拉低 hwgc_lower_irq
// 检查是否启用 msi function
static bool hwgc_msi_enabled(HWGCState *hwgc) { return msi_enabled(&hwgc->pdev); }
static void hwgc_raise_irq(HWGCState *hwgc, uint32_t val)
{
    qatomic_or(&hwgc->irq_status, val);
    if (hwgc->irq_status)
    {
        if (hwgc_msi_enabled(hwgc))
            msi_notify(&hwgc->pdev, 0);
        else
            pci_set_irq(&hwgc->pdev, 1);
    }
}
static void hwgc_lower_irq(HWGCState *hwgc, uint32_t val)
{
    qatomic_and(&hwgc->irq_status, ~val);
    if (!hwgc->irq_status && !hwgc_msi_enabled(hwgc))
        pci_set_irq(&hwgc->pdev, 0);
}

static inline unsigned int hwgc_tlb_hash(uintptr_t va_page) { return va_page >> (TARGET_PAGE_BITS) & (HWGC_TLB_SIZE - 1); }
static inline void flush_hwgc_tlb(HWGCState *hwgc) { memset(hwgc->tlb_cache, 0, HWGC_TLB_SIZE * sizeof(HWGCTLBEntry)); }

static hwaddr hwgc_translate_va(HWGCState *hwgc, uintptr_t vaddr)
{
    uintptr_t va_page = vaddr & TARGET_PAGE_MASK;
    unsigned int idx = hwgc_tlb_hash(va_page);
    HWGCTLBEntry *entry = &hwgc->tlb_cache[idx];

    if (entry->va_page == va_page)
        return entry->pa_page | (vaddr & ~TARGET_PAGE_MASK);

    CPUState *cpu = hwgc->cpu;
    hwaddr pa_page = cpu_get_phys_page_debug(cpu, va_page);
    if (pa_page == (hwaddr)-1 || pa_page == va_page)
        return (hwaddr)-1;

    entry->va_page = va_page;
    entry->pa_page = pa_page;

    return pa_page | (vaddr & ~TARGET_PAGE_MASK);
}

static int access_hwaddr(HWGCState *hwgc, uintptr_t va, void *value, int size, bool write, const char *debug_info)
{
    hwaddr pa = hwgc_translate_va(hwgc, va);

    if (pa == (hwaddr)-1)
        return -1;

    uint8_t buf[8] = {0};

    if (write)
    {
        memcpy(buf, value, size);
        address_space_write(hwgc->cpu->as, pa, MEMTXATTRS_UNSPECIFIED, buf, size);
    }
    else
    {
        address_space_read(hwgc->cpu->as, pa, MEMTXATTRS_UNSPECIFIED, buf, size);
        memcpy(value, buf, size);
    }

    return 0;
}
static bool safeAccessHWAddr(HWGCState *hwgc, uintptr_t addr, void *data, int size, const char *debug_info, bool write, enum HWGC_EXEC_STEP state, int sub_state)
{
    int ret = access_hwaddr(hwgc, addr, data, size, write, debug_info);

    if (ret == -1)
    {
        uint64_t value = 0;
        if (size <= 8 && size > 0)
            memcpy(&value, data, size);

        if (qatomic_read(&hwgc->status) & HWGC_STATUS_IRQ)
        {
            bql_lock();
            hwgc->irq_pars.par0 = addr;
            hwgc->irq_pars.par1 = value;
            hwgc->irq_pars.par2 = write;
            hwgc->irq_pars.par3 = size;
            hwgc->irq_pars.obj_ptr = (uintptr_t)data;
            hwgc->wake_state = state;
            hwgc->wake_sub_state = sub_state;
            hwgc->state = STEP_PAGE_FAULT;
            hwgc_raise_irq(hwgc, PAGE_FAULT_IRQ);
            bql_unlock();
        }
        return false;
    }
    return true;
}

static int my_cmpxchg(HWGCState *hwgc, uintptr_t vaddr, uint64_t old_val, uint64_t new_val, int size)
{
    hwaddr paddr = hwgc_translate_va(hwgc, vaddr);
    if (paddr == (hwaddr)-1)
    {
        printf("translate failed\n");
        return -2;
    }
    hwaddr xlat = paddr;
    hwaddr len = size;
    MemoryRegion *mr = address_space_translate(hwgc->cpu->as, paddr, &xlat, &len, true, MEMTXATTRS_UNSPECIFIED);
    if (!memory_region_is_ram(mr) || len < size)
    {
        printf("memory region not is ram\n");
        return -1;
    }

    uint8_t *host_base = memory_region_get_ram_ptr(mr);
    if (size == 8)
    {
        uint64_t *host_p = (uint64_t *)(host_base + xlat);
        uint64_t seen = qatomic_cmpxchg(host_p, old_val, new_val);
        if (seen == old_val)
        {
            memory_region_set_dirty(mr, xlat, sizeof(uint64_t));
            return 1;
        }
        else
            return 0;
    }
    else if (size == 4)
    {
        uint *host_p = (uint *)(host_base + xlat);
        uint seen = qatomic_cmpxchg(host_p, (uint)old_val, (uint)new_val);
        if (seen == (uint)old_val)
        {
            memory_region_set_dirty(mr, xlat, sizeof(uint));
            return 1;
        }
        else
            return 0;
    }
    else
        return 0;
}

static uint64_t hwgc_mmio_read(void *opaque, hwaddr addr, unsigned size)
{
    HWGCState *hwgc = opaque;
    if (size != 4 && size != 8)
        return ~0ULL;

    switch (addr)
    {
    case REG_STATUS:
        return qatomic_read(&hwgc->status);

    case REG_IRQ_STATUS:
        return qatomic_read(&hwgc->irq_status);

    case REG_IRQ_PAR0:
        return hwgc->irq_pars.par0;

    case REG_IRQ_PAR1:
        return hwgc->irq_pars.par1;

    case REG_IRQ_PAR2:
        return hwgc->irq_pars.par2;

    case REG_IRQ_PAR3:
        return hwgc->irq_pars.par3;

    case REG_IRQ_RES0:
        return hwgc->irq_pars.obj_ptr;

    case REG_IRQ_RES1:
        return hwgc->irq_pars.actual_word_size;

    default:
        return ~0ULL;
    }
}

static void hwgc_mmio_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    HWGCState *hwgc = opaque;

    // handler don't need lock
    if (addr == REG_STATUS && size == 4)
    {
        if (val & HWGC_STATUS_IRQ)
        {
            qatomic_or(&hwgc->status, HWGC_STATUS_IRQ);
            smp_mb__after_rmw();
        }
        else
            qatomic_and(&hwgc->status, ~HWGC_STATUS_IRQ);
        return;
    }

    if (addr == REG_CLEAR_IRQ && size == 4)
    {
        hwgc_lower_irq(hwgc, val);
        return;
    }

    qemu_mutex_lock(&hwgc->thr_mutex);
    if (addr >= REG_PAR0 && addr <= REG_PAR6)
    {
        if (qatomic_read(&hwgc->status) & HWGC_STATUS_COMPUTING || size != 8)
            goto _return;
        static const size_t par_offsets[] = {
            offsetof(struct HWGC_PARALLOCATE_PARS, dest_attr_type),
            offsetof(struct HWGC_PARALLOCATE_PARS, allocator_ptr),
            offsetof(struct HWGC_PARALLOCATE_PARS, alloc_region),
            offsetof(struct HWGC_PARALLOCATE_PARS, min_word_size),
            offsetof(struct HWGC_PARALLOCATE_PARS, desired_word_size),
            offsetof(struct HWGC_PARALLOCATE_PARS, freelist_lock_ptr),
            offsetof(struct HWGC_PARALLOCATE_PARS, thread),
        };
        int idx = (addr - REG_PAR0) / 8;
        uint8_t *base = (uint8_t *)&hwgc->pars;
        *(uint64_t *)(base + par_offsets[idx]) = val;
    }

    if (addr == REG_START_WORK)
    {
        hwgc->cpu = current_cpu;
        hwgc->state = STEP_PAR_ALLOCATE;

        qatomic_or(&hwgc->status, HWGC_STATUS_COMPUTING);
        qemu_cond_signal(&hwgc->thr_cond);
    }

    if (addr == REG_CONTINUE_WORK)
    {
        qatomic_or(&hwgc->status, HWGC_STATUS_WAKE);
        qemu_cond_signal(&hwgc->thr_cond);
    }

    if (addr == REG_IRQ_RES0)
    {
        if (hwgc->state == STEP_PAGE_FAULT && (hwgc->irq_pars.par2 & 0xff) == 0x0)
            memcpy((void *)hwgc->irq_pars.obj_ptr, (void *)&val, hwgc->irq_pars.par3);
        hwgc->irq_pars.obj_ptr = val;
    }

    if (addr == REG_IRQ_RES1)
        hwgc->irq_pars.actual_word_size = val;

_return:
    qemu_mutex_unlock(&hwgc->thr_mutex);
}

static const MemoryRegionOps hwgc_mmio_ops = {
    .read = hwgc_mmio_read,
    .write = hwgc_mmio_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 8,
    },
    .impl = {
        .min_access_size = 4,
        .max_access_size = 8,
    },
};

static void do_par_allocate_iml(void *opaque)
{
    HWGCState *hwgc = opaque;

    if (hwgc->state == STEP_PAR_ALLOCATE)
    {
        if (hwgc->sub_state == 0)
        {
            hwgc->result = 0;
            hwgc->actual_word_size = 0;
            if (hwgc->pars.dest_attr_type == 0)
            {
                hwgc->state = STEP_ALLOCATE_IML;
                hwgc->sub_state = 0;
                hwgc->par_allocate_iml_sel = 0;
            }
            else if (hwgc->pars.dest_attr_type == 1)
            {
                hwgc->state = STEP_ALLOCATE;
                hwgc->sub_state = 0;
                hwgc->par_allocate_sel = 0;
            }
        }

        if (hwgc->sub_state == 1)
        {
            if (hwgc->result == 0)
            {
                hwgc->access_ok = safeAccessHWAddr(hwgc, hwgc->pars.allocator_ptr + 0x10, &hwgc->is_full_value, 8, "read allocator_ptr + 0x10", false, STEP_PAR_ALLOCATE, 2);
                if (hwgc->access_ok)
                    hwgc->sub_state = 2;
            }
            else
                hwgc->sub_state = 10;
        }

        if (hwgc->sub_state == 2)
        {
            bool is_full = hwgc->pars.dest_attr_type == 0 ? hwgc->is_full_value & 0x1 : hwgc->is_full_value & 0x2;
            if (!is_full)
                hwgc->sub_state = 3;
            else
                hwgc->sub_state = 10;
        }

        if (hwgc->sub_state == 3)
        {
            if (hwgc->pars.dest_attr_type == 0)
            {
                hwgc->state = STEP_ALLOCATE_IML;
                hwgc->sub_state = 0;
                hwgc->par_allocate_iml_sel = 1;
            }
            else if (hwgc->pars.dest_attr_type == 1)
            {
                hwgc->state = STEP_ALLOCATE;
                hwgc->sub_state = 0;
                hwgc->par_allocate_sel = 1;
            }
        }

        if (hwgc->sub_state == 4)
        {
            if (hwgc->result == 0)
                hwgc->sub_state = 5;
            else
                hwgc->sub_state = 10;
        }

        if (hwgc->sub_state == 5)
        {
            uintptr_t lock_ptr = hwgc->pars.freelist_lock_ptr;
            int signal = my_cmpxchg(hwgc, lock_ptr + 8, 0, 1, 4);
            if (signal == -1)
                assert(0);
            else if (signal == -2)
            {
                if (qatomic_read(&hwgc->status) & HWGC_STATUS_IRQ)
                {
                    bql_lock();
                    hwgc->irq_pars.par0 = lock_ptr + 8;
                    hwgc->irq_pars.par1 = 0;
                    hwgc->irq_pars.par2 = 1;
                    hwgc->irq_pars.par3 = 4;
                    hwgc->wake_state = STEP_PAR_ALLOCATE;
                    hwgc->wake_sub_state = 11;
                    hwgc->state = STEP_ATOMIC_IRQ;
                    hwgc_raise_irq(hwgc, ATOMIC_IRQ);
                    bql_unlock();
                    return;
                }
            }
            else if (signal == 1)
            {
                hwgc->access_ok = safeAccessHWAddr(hwgc, lock_ptr, &hwgc->pars.thread, 8, "write thread", true, STEP_PAR_ALLOCATE, 6);
                if (hwgc->access_ok)
                    hwgc->sub_state = 6;
            }
            else
                hwgc->sub_state = 5;
        }

        if (hwgc->sub_state == 11)
        {
            if ((uint)hwgc->irq_pars.obj_ptr == 0)
            {
                uintptr_t lock_ptr = hwgc->pars.freelist_lock_ptr;
                hwgc->access_ok = safeAccessHWAddr(hwgc, lock_ptr, &hwgc->pars.thread, 8, "write thread", true, STEP_PAR_ALLOCATE, 6);
                if (hwgc->access_ok)
                    hwgc->sub_state = 6;
            }
            else
                hwgc->sub_state = 5;
        }

        if (hwgc->sub_state == 6)
        {
            if (qatomic_read(&hwgc->status) & HWGC_STATUS_IRQ)
            {
                bql_lock();
                hwgc->wake_state = STEP_PAR_ALLOCATE;
                hwgc->wake_sub_state = 7;
                hwgc->state = STEP_ATTEMPT_IRQ;
                hwgc_raise_irq(hwgc, ATTEMPT_IRQ);
                bql_unlock();
                return;
            }
        }

        if (hwgc->sub_state == 7)
        {
            hwgc->result = hwgc->irq_pars.obj_ptr;
            hwgc->actual_word_size = hwgc->irq_pars.actual_word_size;
            if (hwgc->result == 0)
            {
                uintptr_t addr = hwgc->pars.allocator_ptr + (hwgc->pars.dest_attr_type ? 0x11 : 0x10);
                uint data = 1;
                hwgc->access_ok = safeAccessHWAddr(hwgc, addr, &data, 1, "write full value", true, STEP_PAR_ALLOCATE, 8);
                if (hwgc->access_ok)
                    hwgc->sub_state = 8;
            }
            else
                hwgc->sub_state = 8;
        }

        if (hwgc->sub_state == 8)
        {
            uintptr_t lock_ptr = hwgc->pars.freelist_lock_ptr;
            hwgc->access_ok = safeAccessHWAddr(hwgc, lock_ptr + 8, &hwgc->wait_num, 4, "read wait num", false, STEP_PAR_ALLOCATE, 9);
            if (hwgc->access_ok)
                hwgc->sub_state = 9;
        }

        if (hwgc->sub_state == 9)
        {
            if (hwgc->wait_num > 1)
            {
                if (qatomic_read(&hwgc->status) & HWGC_STATUS_IRQ)
                {

                    bql_lock();
                    hwgc->irq_pars.par0 = hwgc->pars.freelist_lock_ptr;
                    hwgc->wake_state = STEP_PAR_ALLOCATE;
                    hwgc->wake_sub_state = 10;
                    hwgc->state = STEP_LOCK_WAKE;
                    hwgc_raise_irq(hwgc, LOCK_WAKE_IRQ);
                    bql_unlock();
                    return;
                }
            }
            else
            {
                uint num = 0;
                uintptr_t lock_ptr = hwgc->pars.freelist_lock_ptr;
                hwgc->access_ok = safeAccessHWAddr(hwgc, lock_ptr + 8, &num, 4, "write wait num", true, STEP_PAR_ALLOCATE, 10);
                if (hwgc->access_ok)
                    hwgc->sub_state = 10;
            }
        }

        if (hwgc->sub_state == 10)
        {
            hwgc->irq_pars.obj_ptr = hwgc->result;
            hwgc->irq_pars.actual_word_size = hwgc->actual_word_size;
            hwgc->state = STEP_DONE;
            hwgc->sub_state = 0;
        }
    }

    if (hwgc->state == STEP_ALLOCATE_IML)
    {
        if (hwgc->sub_state == 0)
        {
            hwgc->access_ok = safeAccessHWAddr(hwgc, hwgc->pars.alloc_region + 0x10, &hwgc->alloc_top, 8, "read alloc_region + 0x10", false, STEP_ALLOCATE_IML, 1);
            if (hwgc->access_ok)
                hwgc->sub_state = 1;
        }

        if (hwgc->sub_state == 1)
        {
            hwgc->access_ok = safeAccessHWAddr(hwgc, hwgc->pars.alloc_region + 0x8, &hwgc->alloc_end, 8, "read alloc_region + 0x8", false, STEP_ALLOCATE_IML, 2);
            if (hwgc->access_ok)
                hwgc->sub_state = 2;
        }

        if (hwgc->sub_state == 2)
        {
            hwgc->alloc_available = (hwgc->alloc_end - hwgc->alloc_top) / 8;
            hwgc->want_to_allocate = hwgc->alloc_available > hwgc->pars.desired_word_size ? hwgc->pars.desired_word_size : hwgc->alloc_available;
            if (hwgc->want_to_allocate >= hwgc->pars.min_word_size)
            {
                int signal = my_cmpxchg(hwgc, hwgc->pars.alloc_region + 0x10, hwgc->alloc_top, hwgc->alloc_top + hwgc->want_to_allocate * 8, 8);
                if (signal == -1)
                    assert(0);
                else if (signal == -2)
                {
                    if (qatomic_read(&hwgc->status) & HWGC_STATUS_IRQ)
                    {
                        bql_lock();
                        hwgc->irq_pars.par0 = hwgc->pars.alloc_region + 0x10;
                        hwgc->irq_pars.par1 = hwgc->alloc_top;
                        hwgc->irq_pars.par2 = hwgc->alloc_top + hwgc->want_to_allocate * 8;
                        hwgc->irq_pars.par3 = 8;
                        hwgc->wake_state = STEP_ALLOCATE_IML;
                        hwgc->wake_sub_state = 4;
                        hwgc->state = STEP_ATOMIC_IRQ;
                        hwgc_raise_irq(hwgc, ATOMIC_IRQ);
                        bql_unlock();
                        return;
                    }
                }
                else if (signal)
                    hwgc->sub_state = 3;
                else
                    hwgc->sub_state = 0;
            }
            else
            {
                hwgc->result = 0;
                hwgc->actual_word_size = 0;
                hwgc->state = hwgc->par_allocate_iml_sel == 2 ? STEP_ALLOCATE : STEP_PAR_ALLOCATE;
                hwgc->sub_state = hwgc->par_allocate_iml_sel == 2 ? 2 : hwgc->par_allocate_iml_sel == 1 ? 4
                                                                                                        : 1;
                return;
            }
        }

        if (hwgc->sub_state == 4)
        {
            if (hwgc->irq_pars.obj_ptr == hwgc->alloc_top)
                hwgc->sub_state = 3;
            else
                hwgc->sub_state = 0;
        }

        if (hwgc->sub_state == 3)
        {
            hwgc->result = hwgc->alloc_top;
            hwgc->actual_word_size = hwgc->want_to_allocate;
            hwgc->state = hwgc->par_allocate_iml_sel == 2 ? STEP_ALLOCATE : STEP_PAR_ALLOCATE;
            hwgc->sub_state = hwgc->par_allocate_iml_sel == 2 ? 2 : hwgc->par_allocate_iml_sel == 1 ? 4
                                                                                                    : 1;
        }
    }

    if (hwgc->state == STEP_ALLOCATE)
    {
        if (hwgc->sub_state == 0)
        {
            uintptr_t lock_ptr = hwgc->pars.alloc_region + 0x40;
            int signal = my_cmpxchg(hwgc, lock_ptr + 8, 0, 1, 4);
            if (signal == -1)
                assert(0);
            else if (signal == -2)
            {
                if (qatomic_read(&hwgc->status) & HWGC_STATUS_IRQ)
                {
                    bql_lock();
                    hwgc->irq_pars.par0 = lock_ptr + 8;
                    hwgc->irq_pars.par1 = 0;
                    hwgc->irq_pars.par2 = 1;
                    hwgc->irq_pars.par3 = 4;
                    hwgc->wake_state = STEP_ALLOCATE;
                    hwgc->wake_sub_state = 6;
                    hwgc->state = STEP_ATOMIC_IRQ;
                    hwgc_raise_irq(hwgc, ATOMIC_IRQ);
                    bql_unlock();
                    return;
                }
            }
            else if (signal == 1)
                hwgc->sub_state = 1;
            else
                hwgc->sub_state = 0;
        }

        if (hwgc->sub_state == 6)
        {
            if ((uint)hwgc->irq_pars.obj_ptr == 0)
                hwgc->sub_state = 1;
            else
                hwgc->sub_state = 0;
        }

        if (hwgc->sub_state == 1)
        {
            hwgc->state = STEP_ALLOCATE_IML;
            hwgc->sub_state = 0;
            hwgc->par_allocate_iml_sel = 2;
        }

        if (hwgc->sub_state == 2)
        {
            if (qatomic_read(&hwgc->status) & HWGC_STATUS_IRQ)
            {
                bql_lock();
                hwgc->irq_pars.par0 = hwgc->result;
                hwgc->irq_pars.par1 = hwgc->actual_word_size;
                hwgc->wake_state = STEP_ALLOCATE;
                hwgc->wake_sub_state = 3;
                hwgc->state = STEP_ALLOCATE_IRQ;
                hwgc_raise_irq(hwgc, ALLOCATE_IRQ);
                bql_unlock();
                return;
            }
        }

        if (hwgc->sub_state == 3)
        {
            uintptr_t lock_ptr = hwgc->pars.alloc_region + 0x40;
            hwgc->access_ok = safeAccessHWAddr(hwgc, lock_ptr + 8, &hwgc->wait_num, 4, "read wait num", false, STEP_ALLOCATE, 3);
            if (hwgc->access_ok)
                hwgc->sub_state = 4;
        }

        if (hwgc->sub_state == 4)
        {
            if (hwgc->wait_num > 1)
            {
                if (qatomic_read(&hwgc->status) & HWGC_STATUS_IRQ)
                {
                    bql_lock();
                    hwgc->irq_pars.par0 = hwgc->pars.alloc_region + 0x40;
                    hwgc->wake_state = STEP_ALLOCATE;
                    hwgc->wake_sub_state = 5;
                    hwgc->state = STEP_LOCK_WAKE;
                    hwgc_raise_irq(hwgc, LOCK_WAKE_IRQ);
                    bql_unlock();
                    return;
                }
            }
            else
            {
                uint num = 0;
                uintptr_t lock_ptr = hwgc->pars.alloc_region + 0x40;
                hwgc->access_ok = safeAccessHWAddr(hwgc, lock_ptr + 8, &num, 4, "write wait num", true, STEP_ALLOCATE, 5);
                if (hwgc->access_ok)
                    hwgc->sub_state = 5;
            }
        }

        if (hwgc->sub_state == 5)
        {
            hwgc->state = STEP_PAR_ALLOCATE;
            hwgc->sub_state = hwgc->par_allocate_sel == 1 ? 4 : 1;
        }
    }
}

static void *hwgc_work_thread(void *opaque)
{
    HWGCState *hwgc = opaque;

    while (1)
    {
        qemu_mutex_lock(&hwgc->thr_mutex);
        while ((qatomic_read(&hwgc->status) & HWGC_STATUS_COMPUTING) == 0)
            qemu_cond_wait(&hwgc->thr_cond, &hwgc->thr_mutex);

        qemu_mutex_unlock(&hwgc->thr_mutex);

        printf("do hwgc work\n");

        while (1)
        {
            do_par_allocate_iml(hwgc);
            if (hwgc->state == STEP_DONE)
                break;
            if (hwgc->state == STEP_LOCK_WAKE || hwgc->state == STEP_ALLOCATE_IRQ || hwgc->state == STEP_ATTEMPT_IRQ || hwgc->state == STEP_PAGE_FAULT || hwgc->state == STEP_ATOMIC_IRQ)
            {
                qemu_mutex_lock(&hwgc->thr_mutex);
                while ((qatomic_read(&hwgc->status) & HWGC_STATUS_WAKE) == 0)
                    qemu_cond_wait(&hwgc->thr_cond, &hwgc->thr_mutex);

                hwgc->state = hwgc->wake_state;
                hwgc->sub_state = hwgc->wake_sub_state;
                qatomic_and(&hwgc->status, ~HWGC_STATUS_WAKE);
                qemu_mutex_unlock(&hwgc->thr_mutex);
            }
        }

        printf("do hwgc end\n");

        qatomic_and(&hwgc->status, ~HWGC_STATUS_COMPUTING);
        smp_mb__after_rmw();
        if (qatomic_read(&hwgc->status) & HWGC_STATUS_IRQ)
        {
            bql_lock();
            hwgc_raise_irq(hwgc, COMPLETE_IRQ);
            bql_unlock();
        }
    }
    return NULL;
}

static void pci_hwgc_realize(PCIDevice *pdev, Error **errp)
{
    HWGCState *hwgc = HWGC(pdev);
    flush_hwgc_tlb(hwgc);

    uint8_t *pci_conf = pdev->config;

    pci_config_set_interrupt_pin(pci_conf, 1); // 注册中断 使用INTx的引脚1

    // offset = 0: 自动分配
    // nr_vectors = 1: 支持一个MSI向量, 该设备只需要一个中断位
    // msi64bit = true: 现代系统都需要支持64位地址
    // msi_per_vector_mask: false
    // 系统不支持msi 会返回
    if (msi_init(pdev, 0, 1, true, false, errp))
        return;

    // 创建后台线程
    qemu_mutex_init(&hwgc->thr_mutex);
    qemu_cond_init(&hwgc->thr_cond);
    qemu_thread_create(&hwgc->thread, "hwgc", hwgc_work_thread, hwgc, QEMU_THREAD_JOINABLE);

    // 注册 1MB 的 MMIO 并 映射到 PCI BAR 0
    memory_region_init_io(&hwgc->mmio, OBJECT(hwgc), &hwgc_mmio_ops, hwgc, "hwgc-mmio", 1 * MiB);
    pci_register_bar(pdev, 0, PCI_BASE_ADDRESS_SPACE_MEMORY, &hwgc->mmio);
}

static void pci_hwgc_uninit(PCIDevice *pdev)
{
    HWGCState *hwgc = HWGC(pdev);

    qemu_cond_signal(&hwgc->thr_cond);
    qemu_thread_join(&hwgc->thread);

    qemu_cond_destroy(&hwgc->thr_cond);
    qemu_mutex_destroy(&hwgc->thr_mutex);

    msi_uninit(pdev);
}

static void hwgc_instance_init(Object *obj)
{
    HWGCState *hwgc = HWGC(obj);
}

static void hwgc_class_init(ObjectClass *class, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(class);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(class);

    k->realize = pci_hwgc_realize;
    k->exit = pci_hwgc_uninit;
    k->vendor_id = PCI_VENDOR_ID_QEMU; // 厂商ID, 0x1234是QEMU官方保留的测试厂商ID
    k->device_id = 0x0308;             // 设备ID
    /* Linux驱动通过(vendor_id, device_id)唯一标识设备
       static const struct pci_device_id hwgc_pci_ids[] = {
           { PCI_DEVICE(PCI_VENDOR_ID_QEMU, 0x11e8) },
           { }
       };
       MODULE_DEVICE_TABLE(pci, hwgc_pci_ids);
    */
    k->revision = 0x2;                             // 设备的修订版本号
    k->class_id = PCI_CLASS_OTHERS;                // 设备类别 PCI_CLASS_OTHERS属于未分类类别
    set_bit(DEVICE_CATEGORY_MISC, dc->categories); // 分组 在MISC组内显示 qemu -device help
}

static const TypeInfo hwgc_types[] = {
    {
        .parent = TYPE_PCI_DEVICE,
        .name = TYPE_PCI_HWGC_DEVICE,
        .class_init = hwgc_class_init,
        .instance_size = sizeof(HWGCState),
        .instance_init = hwgc_instance_init,
        .interfaces = (const InterfaceInfo[]){
            {INTERFACE_CONVENTIONAL_PCI_DEVICE},
            {},
        },
    }};

DEFINE_TYPES(hwgc_types)