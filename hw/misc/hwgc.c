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
    uint32_t wake_state;

    // data
    bool access_ok;
    uintptr_t alloc_top, alloc_end;
    size_t alloc_available, want_to_allocate;

    // pars
    struct HWGC_PARALLOCATE_PARS pars;

    // irq pars and res
    struct HWGC_IRQ_PARS irq_pars;

    CPUState *cpu;
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

static hwaddr hwgc_translate_va(HWGCState *hwgc, uintptr_t vaddr)
{
    uintptr_t va_page = vaddr & TARGET_PAGE_MASK;
    CPUState *cpu = hwgc->cpu;
    hwaddr pa_page = cpu_get_phys_page_debug(cpu, va_page);

    if (pa_page == (hwaddr)-1)
        // Handle invalid translation
        return (hwaddr)-1;
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
static bool safeAccessHWAddr(HWGCState *hwgc, uintptr_t addr, void *data, int size, const char *debug_info, bool write, enum HWGC_EXEC_STEP next)
{
    int ret = access_hwaddr(hwgc, addr, data, size, write, debug_info);

    if (ret == -1)
    {
        uint64_t value = 0;
        if (size <= 8 && size > 0)
            memcpy(&value, data, size);

        hwgc->irq_pars.par0 = addr;
        hwgc->irq_pars.par1 = value;
        hwgc->irq_pars.par2 = write;
        hwgc->irq_pars.par3 = size;
        hwgc->irq_pars.obj_ptr = (uintptr_t)data;
        hwgc->wake_state = next;
        hwgc->state = STEP_PAGE_FAULT;
        if (qatomic_read(&hwgc->status) & HWGC_STATUS_IRQ)
        {
            bql_lock();
            hwgc_raise_irq(hwgc, PAGE_FAULT_IRQ);
            bql_unlock();
        }
        return false;
    }
    return true;
}

static int my_cmpxchg(HWGCState *hwgc, uintptr_t vaddr, uint64_t old_val, uint64_t new_val)
{
    hwaddr paddr = hwgc_translate_va(hwgc, vaddr);
    hwaddr xlat = paddr;
    hwaddr len = sizeof(uint64_t);
    MemoryRegion *mr = address_space_translate(hwgc->cpu->as, paddr, &xlat, &len, true, MEMTXATTRS_UNSPECIFIED);
    if (!memory_region_is_ram(mr) || len < sizeof(uint64_t))
    {
        printf("memory region not is ram");
        return -1;
    }

    uint8_t *host_base = memory_region_get_ram_ptr(mr);
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
        return hwgc->irq_pars.actual_plab_size;

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
    if (addr >= REG_PAR0 && addr <= REG_PAR2)
    {
        if (qatomic_read(&hwgc->status) & HWGC_STATUS_COMPUTING || size != 8)
            goto _return;
        static const size_t par_offsets[] = {
            offsetof(struct HWGC_PARALLOCATE_PARS, alloc_region),      // REG_PAR0: lo=chunkSize, hi=ageThreshold
            offsetof(struct HWGC_PARALLOCATE_PARS, min_word_size),     // REG_PAR1: lo=heapRegionBias, hi=regionAttrShiftBy
            offsetof(struct HWGC_PARALLOCATE_PARS, desired_word_size), // REG_PAR2: lo=heapRegionShiftBy, hi=logOfHRGrainBytes
        };
        int idx = (addr - REG_PAR0) / 8;
        uint8_t *base = (uint8_t *)&hwgc->pars;
        *(uint64_t *)(base + par_offsets[idx]) = val;
    }

    if (addr == REG_START_WORK)
    {
        hwgc->cpu = current_cpu;
        hwgc->state = STEP_ACCESS_TOP;

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

    if (hwgc->state == STEP_ACCESS_TOP)
    {
        hwgc->access_ok = safeAccessHWAddr(hwgc, hwgc->pars.alloc_region + 0x10, &hwgc->alloc_top, 8, "read alloc_region + 0x10", false, STEP_ACCESS_END);
        if (hwgc->access_ok)
        {
            printf("alloc_top %lx\n", hwgc->alloc_top);
            hwgc->state = STEP_ACCESS_END;
        }
    }

    if (hwgc->state == STEP_ACCESS_END)
    {
        hwgc->access_ok = safeAccessHWAddr(hwgc, hwgc->pars.alloc_region + 0x8, &hwgc->alloc_end, 8, "read alloc_region + 0x8", false, STEP_BRANCH);
        if (hwgc->access_ok)
        {
            printf("alloc_end %lx\n", hwgc->alloc_end);
            hwgc->state = STEP_BRANCH;
        }
    }

    if (hwgc->state == STEP_BRANCH)
    {
        hwgc->alloc_available = (hwgc->alloc_end - hwgc->alloc_top) / 8;
        hwgc->want_to_allocate = hwgc->alloc_available > hwgc->pars.desired_word_size ? hwgc->pars.desired_word_size : hwgc->alloc_available;
        printf("Branching want %lx desired %lx available %lx\n", hwgc->want_to_allocate, hwgc->pars.desired_word_size, hwgc->alloc_available);
        if (hwgc->want_to_allocate >= hwgc->pars.min_word_size)
        {
            int signal = my_cmpxchg(hwgc, hwgc->pars.alloc_region + 0x10, hwgc->alloc_top, hwgc->alloc_top + hwgc->want_to_allocate * 8);
            if (signal == -1)
            {
                hwgc->irq_pars.par0 = hwgc->pars.alloc_region + 0x10;
                hwgc->irq_pars.par1 = hwgc->alloc_top;
                hwgc->irq_pars.par2 = hwgc->alloc_top + hwgc->want_to_allocate * 8;
                hwgc->wake_state = STEP_ATOMIC_RESULT;
                hwgc->state = STEP_ATOMIC;
                if (qatomic_read(&hwgc->status) & HWGC_STATUS_IRQ)
                {
                    bql_lock();
                    hwgc_raise_irq(hwgc, ATOMIC_IRQ);
                    bql_unlock();
                }
            }
            else if (signal)
            {
                hwgc->irq_pars.obj_ptr = hwgc->alloc_top;
                hwgc->irq_pars.actual_plab_size = hwgc->want_to_allocate;
                hwgc->state = STEP_DONE;
            }
            else
                hwgc->state = STEP_ACCESS_TOP;
        }
        else
        {
            hwgc->irq_pars.obj_ptr = 0;
            hwgc->irq_pars.actual_plab_size = 0;
            hwgc->state = STEP_DONE;
        }
    }

    if (hwgc->state == STEP_ATOMIC_RESULT)
    {
        printf("Atomic result %lx %lx\n", hwgc->irq_pars.obj_ptr, hwgc->alloc_top);
        if (hwgc->irq_pars.obj_ptr == hwgc->alloc_top)
        {
            hwgc->irq_pars.obj_ptr = hwgc->alloc_top;
            hwgc->irq_pars.actual_plab_size = hwgc->want_to_allocate;
            hwgc->state = STEP_DONE;
        }
        else
            hwgc->state = STEP_ACCESS_TOP;
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
            if (hwgc->state == STEP_ATOMIC)
            {
                qemu_mutex_lock(&hwgc->thr_mutex);
                while ((qatomic_read(&hwgc->status) & HWGC_STATUS_WAKE) == 0)
                    qemu_cond_wait(&hwgc->thr_cond, &hwgc->thr_mutex);

                hwgc->state = hwgc->wake_state;
                qatomic_and(&hwgc->status, ~HWGC_STATUS_WAKE);
                qemu_mutex_unlock(&hwgc->thr_mutex);
            }

            if (hwgc->state == STEP_PAGE_FAULT)
            {
                qemu_mutex_lock(&hwgc->thr_mutex);
                while ((qatomic_read(&hwgc->status) & HWGC_STATUS_WAKE) == 0)
                    qemu_cond_wait(&hwgc->thr_cond, &hwgc->thr_mutex);

                hwgc->state = hwgc->wake_state;
                qatomic_and(&hwgc->status, ~HWGC_STATUS_WAKE);
                qemu_mutex_unlock(&hwgc->thr_mutex);
            }
        }

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