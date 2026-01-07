#include "hwgc.h"
#include "qemu/xxhash.h"

#define TYPE_PCI_HWGC_DEVICE "hwgc"
typedef struct HWGCState HWGCState;
DECLARE_INSTANCE_CHECKER(HWGCState, HWGC, TYPE_PCI_HWGC_DEVICE)

#define HWGC_TLB_SIZE 1048576

typedef struct
{
    vaddr va_page;
    hwaddr pa_page;
} HWGCTLBEntry;

struct HWGCState
{
    PCIDevice pdev;
    MemoryRegion mmio;

    QemuThread thread;
    QemuMutex thr_mutex;
    QemuCond thr_cond;

    // reg data
    uint32_t status;
    uint32_t irq_status;

    enum HWGC_EXEC_STEP state;
    enum HWGC_EXEC_STEP wake_state;
    enum HWGC_EXEC_STEP previous;
    enum HWGC_EXEC_STEP done_to;

    int sub_state;
    int wake_sub_state;
    int doneto_sub_state;
    int previous_sub_state;

    struct HWGCParameter pars;
    struct HWGCSoftRelated softPars;

    CPUState *cpu;
    HWGCTLBEntry tlb_cache[HWGC_TLB_SIZE];
};

static uint64_t get_device_id(HWGCState *s) { return HWGC_DEVICE_ID; }
static uint64_t get_status(HWGCState *s) { return qatomic_read(&s->status); }
static uint64_t get_int_status(HWGCState *s) { return s->irq_status; }
static uint64_t get_soft_par0(HWGCState *s) { return s->softPars.par0; }
static uint64_t get_soft_par1(HWGCState *s) { return s->softPars.par1; }
static uint64_t get_soft_par2(HWGCState *s) { return s->softPars.par2; }
static uint64_t get_soft_par3(HWGCState *s) { return s->softPars.par3; }

static inline unsigned int hwgc_tlb_hash(uintptr_t va_page)
{
    // return qemu_xxhash2((va_page >> 12)) % (HWGC_TLB_SIZE);
    return (va_page >> 12) & 0xfffff;
}
static inline void flush_hwgc_tlb(HWGCState *hwgc)
{
    memset(hwgc->tlb_cache, 0, HWGC_TLB_SIZE * sizeof(HWGCTLBEntry));
}
static hwaddr hwgc_translate_va(HWGCState *hwgc, uintptr_t va)
{
    uintptr_t va_page = va & TARGET_PAGE_MASK;
    unsigned int idx = hwgc_tlb_hash(va_page);
#ifdef DEBUG_ENABLE
    printf("the va page %lx the idx is %d\n", va_page, idx);
#endif
    HWGCTLBEntry *entry = &hwgc->tlb_cache[idx];

    if (entry->va_page == va_page)
        return entry->pa_page | (va & ~TARGET_PAGE_MASK);

    CPUState *cpu = hwgc->cpu;
    hwaddr pa_page = cpu_get_phys_page_debug(cpu, va_page);
    if (pa_page == (hwaddr)-1 || pa_page == va_page)
        return (hwaddr)-1;

    entry->va_page = va_page;
    entry->pa_page = pa_page;

    return pa_page | (va & ~TARGET_PAGE_MASK);
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

// debug
#ifdef DEBUG_ENABLE
    uint64_t print_value;
    memcpy(&print_value, buf, size);
    printf("%s va %lx access %d(write or read) data (size=%d): 0x%lx\n", debug_info, va, write, size, print_value);
#endif

    return 0;
}

// PCI/PCIe 支持两种中断机制
//      MSI: 边沿出发 不需要维持状态 发一次会自动无效掉
//      Legacy INTx: 电平触发 需要手动拉低 hwgc_lower_irq
// 检查是否启用 msi function
static bool hwgc_msi_enabled(HWGCState *hwgc)
{
    return msi_enabled(&hwgc->pdev);
}
static void hwgc_raise_irq(HWGCState *hwgc, uint32_t val)
{
    hwgc->irq_status |= val; // 记录当前有哪些中断 pending
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
    hwgc->irq_status &= ~val;

    if (!hwgc->irq_status && !hwgc_msi_enabled(hwgc))
        pci_set_irq(&hwgc->pdev, 0);
}

static uint64_t hwgc_mmio_read(void *opaque, hwaddr addr, unsigned size)
{
    HWGCState *hwgc = opaque;
    if (size != 4 && size != 8)
        return ~0ULL;
    struct
    {
        hwaddr offset;
        unsigned char width; // 4 or 8
        uint64_t (*get)(HWGCState *);
    } regs[] = {
        {REG_DEVICE_ID, 4, get_device_id},
        {REG_STATUS, 4, get_status},
        {REG_INT_STATUS, 4, get_int_status},
        {REG_SOFT_PAR0, 8, get_soft_par0},
        {REG_SOFT_PAR1, 8, get_soft_par1},
        {REG_SOFT_PAR2, 8, get_soft_par2},
        {REG_SOFT_PAR3, 8, get_soft_par3},
    };
    for (int i = 0; i < ARRAY_SIZE(regs); i++)
    {
        if (regs[i].offset == addr)
        {
            if (size != regs[i].width)
                return ~0ULL;
            return regs[i].get(hwgc);
        }
    }
    return ~0ULL;
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
    if (addr >= REG_PAR0 && addr <= REG_PAR13)
    {
        if (qatomic_read(&hwgc->status) & HWGC_STATUS_COMPUTING || size != 8)
            return;
        static const size_t par_offsets[] = {
            offsetof(struct HWGCParameter, chunkSize),                        // REG_PAR0: lo=chunkSize, hi=ageThreshold
            offsetof(struct HWGCParameter, heapRegionBias),                   // REG_PAR1: lo=heapRegionBias, hi=regionAttrShiftBy
            offsetof(struct HWGCParameter, heapRegionShiftBy),                // REG_PAR2: lo=heapRegionShiftBy, hi=logOfHRGrainBytes
            offsetof(struct HWGCParameter, stepperOffset),                    // REG_PAR3
            offsetof(struct HWGCParameter, youngWordsBase),                   // REG_PAR4
            offsetof(struct HWGCParameter, regionAttrBase),                   // REG_PAR5
            offsetof(struct HWGCParameter, plabAllocatorPtr),                 // REG_PAR6
            offsetof(struct HWGCParameter, regionAttrBiasedBase),             // REG_PAR7
            offsetof(struct HWGCParameter, heapRegionBiasedBase),             // REG_PAR8
            offsetof(struct HWGCParameter, parScanThreadStatePtr),            // REG_PAR9
            offsetof(struct HWGCParameter, taskQueueBottomAddr),              // REG_PAR10
            offsetof(struct HWGCParameter, taskQueueElemsBase),               // REG_PAR11
            offsetof(struct HWGCParameter, humogousReclaimCandidateBoolBase), // REG_PAR12
            offsetof(struct HWGCParameter, cardTablePtr)                      // REG_PAR13
        };
        int idx = (addr - REG_PAR0) / 8;
        uint8_t *base = (uint8_t *)&hwgc->pars;
        if (idx <= 2)
        {
            uint32_t *lo = (uint32_t *)(base + par_offsets[idx]);
            uint32_t *hi = lo + 1;
            *lo = (uint32_t)val;
            *hi = (uint32_t)(val >> 32);
        }
        else
            *(uint64_t *)(base + par_offsets[idx]) = val;
    }

    if (addr == REG_START_WORK)
    {
        hwgc->cpu = current_cpu;
        hwgc->state = STEP_FETCH;
        hwgc->sub_state = 0;

        flush_hwgc_tlb(hwgc);

        qatomic_or(&hwgc->status, HWGC_STATUS_COMPUTING);
        qemu_cond_signal(&hwgc->thr_cond);
    }

    if (addr == REG_CONTINUE_WORK)
    {
        qatomic_or(&hwgc->status, HWGC_STATUS_WAKE);
        qemu_cond_signal(&hwgc->thr_cond);
    }

    if (addr == REG_SOFT_RES)
    {
        if (hwgc->state == STEP_PAGE_FAULT && (hwgc->softPars.par2 & 0xff) == 0x0)
            memcpy((void *)hwgc->softPars.res, (void *)&val, hwgc->softPars.par3);
        hwgc->softPars.res = val;
    }

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

static bool safeAccessHWAddr(HWGCState *hwgc, uintptr_t addr, void *data, int size, const char *debug_info, bool write, enum HWGC_EXEC_STEP prev, int wake_sub_state)
{
    int ret = access_hwaddr(hwgc, addr, data, size, write, debug_info);

    if (ret == -1)
    {
        uint64_t value = 0;
        if (size <= 8 && size > 0)
            memcpy(&value, data, size);

#ifdef DEBUG_ENABLE
        if (write)
            printf("%s va %lx data(size=%d): 0x%lx --- ", debug_info, addr, size, value);
        else
            printf("%s va %lx data (size=%d): --- ", debug_info, addr, size);
#endif

        hwgc->softPars.par0 = addr;
        hwgc->softPars.par1 = value;
        hwgc->softPars.par2 = write;
        hwgc->softPars.par3 = size;
        hwgc->softPars.res = (uintptr_t)data;
        hwgc->wake_state = prev;
        hwgc->wake_sub_state = wake_sub_state;
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

static void do_hwgc_work(void *opaque)
{
    HWGCState *hwgc = opaque;
    bool tag;

    static uintptr_t task;
    static uint fetch_localBot;
    if (hwgc->state == STEP_FETCH)
    {
        if (hwgc->sub_state == 0)
        {
            tag = safeAccessHWAddr(hwgc, hwgc->pars.taskQueueBottomAddr, &fetch_localBot, 4, "read taskqueue bottom addr", false, STEP_FETCH, 1);
            if (tag)
                hwgc->sub_state = 1;
        }

        if (hwgc->sub_state == 1)
        {
            if (fetch_localBot == 0)
            {
                hwgc->state = STEP_DONE;
                hwgc->sub_state = 0;
#ifdef DEBUG_ENABLE
                printf("The jvm taskqueue has handled over, and now enter the state %x\n", STEP_DONE);
#endif
                return;
            }
            else
            {
                fetch_localBot = (fetch_localBot - 1) & ((1 << 17) - 1);
                tag = safeAccessHWAddr(hwgc, hwgc->pars.taskQueueBottomAddr, &fetch_localBot, 4, "write taskqueue bottom", true, STEP_FETCH, 2);
                if (tag)
                    hwgc->sub_state = 2;
            }
        }

        if (hwgc->sub_state == 2)
        {
            tag = safeAccessHWAddr(hwgc, hwgc->pars.taskQueueElemsBase + fetch_localBot * 8, &task, 8, "read taskqueue elems", false, STEP_FETCH, 3);
            if (tag)
                hwgc->sub_state = 3;
        }

        if (hwgc->sub_state == 3)
        {
            hwgc->state = STEP_DISPATCH;
            hwgc->sub_state = 0;
#ifdef DEBUG_ENABLE
            printf("The task is %lx, and now enter the state %x\n", task, hwgc->state);
#endif
        }
    }

    if (hwgc->state == STEP_DISPATCH)
    {
        hwgc->sub_state = 0;
        if ((task & 0x3) == 0x0)
        {
            hwgc->state = STEP_COMMON_OOP;
#ifdef DEBUG_ENABLE
            printf("The task is common oop ptr, now enter the state %x\n", STEP_COMMON_OOP);
#endif
        }
        else
        {
            hwgc->state = STEP_PARTIAL_ARRAY;
#ifdef DEBUG_ENABLE
            printf("The task is partial array oop, now enter the state %x\n", STEP_PARTIAL_ARRAY);
#endif
        }
    }

    static uintptr_t from_obj, to_obj, partial_m_value;
    static int partial_from_length, start;
    static uintptr_t heap_region;
    static uint heap_region_type, ncreate, i;
    static bool scanning_in_young;
    static uintptr_t p, q, src, dest;
    static uint array_localBot;
    if (hwgc->state == STEP_PARTIAL_ARRAY)
    {
        if (hwgc->sub_state == 0)
        {
            from_obj = task - 0x2;
            tag = safeAccessHWAddr(hwgc, from_obj, &partial_m_value, 8, "read partial markword", false, STEP_PARTIAL_ARRAY, 1);
            if (tag)
                hwgc->sub_state = 1;
        }

        if (hwgc->sub_state == 1)
        {
            to_obj = partial_m_value & ~0x3;
            tag = safeAccessHWAddr(hwgc, from_obj + 16, &partial_from_length, 4, "read partial from obj length", false, STEP_PARTIAL_ARRAY, 2);
            if (tag)
                hwgc->sub_state = 2;
        }

        if (hwgc->sub_state == 2)
        {
            tag = safeAccessHWAddr(hwgc, to_obj + 16, &start, 4, "read partial to obj length", false, STEP_PARTIAL_ARRAY, 3);
            if (tag)
                hwgc->sub_state = 3;
        }

        if (hwgc->sub_state == 3)
        {
            int temp = start + hwgc->pars.chunkSize;
            tag = safeAccessHWAddr(hwgc, to_obj + 16, &temp, 4, "write partial to obj length", true, STEP_PARTIAL_ARRAY, 4);
            if (tag)
                hwgc->sub_state = 4;
        }

        if (hwgc->sub_state == 4)
        {
            uint task_num = start / hwgc->pars.chunkSize;
            uint remaining_tasks = (partial_from_length - start) / hwgc->pars.chunkSize;
            uint _task_limit = (uint)hwgc->pars.stepperOffset;
            uint _task_fanout = hwgc->pars.stepperOffset >> 32;
            uint max_pending = (_task_fanout - 1) * task_num + 1;
            uint pending = MIN(max_pending, MIN(remaining_tasks, _task_limit));
            ncreate = MIN(_task_fanout, MIN(remaining_tasks, _task_limit + 1) - pending);

            i = 0;
            hwgc->sub_state = 5;
        }

        if (hwgc->sub_state == 5)
        {
            if (i < ncreate)
            {
                tag = safeAccessHWAddr(hwgc, hwgc->pars.taskQueueBottomAddr, &array_localBot, 4, "read taskqueue bottom addr", false, STEP_PARTIAL_ARRAY, 6);
                if (tag)
                    hwgc->sub_state = 6;
            }
            else
                hwgc->sub_state = 8;
        }

        if (hwgc->sub_state == 6)
        {
            uintptr_t pushData = from_obj + 0x2;
            tag = safeAccessHWAddr(hwgc, hwgc->pars.taskQueueElemsBase + array_localBot * 8, &pushData, 8, "write taskqueue elems", true, STEP_PARTIAL_ARRAY, 7);
            if (tag)
                hwgc->sub_state = 7;
        }

        if (hwgc->sub_state == 7)
        {
            array_localBot = (array_localBot + 1) & ((1 << 17) - 1);
            ++i;
            tag = safeAccessHWAddr(hwgc, hwgc->pars.taskQueueBottomAddr, &array_localBot, 4, "write taskqueue bottom addr", true, STEP_PARTIAL_ARRAY, 5);
            if (tag)
                hwgc->sub_state = 5;
        }

        if (hwgc->sub_state == 8)
        {
            uintptr_t heap_region_ptr = hwgc->pars.heapRegionBiasedBase + (to_obj >> hwgc->pars.heapRegionShiftBy) * 8;
            tag = safeAccessHWAddr(hwgc, heap_region_ptr, &heap_region, 8, "read heap region", false, STEP_PARTIAL_ARRAY, 9);
            if (tag)
                hwgc->sub_state = 9;
        }

        if (hwgc->sub_state == 9)
        {
            tag = safeAccessHWAddr(hwgc, heap_region + 0xbc, &heap_region_type, 4, "read heap region type", false, STEP_PARTIAL_ARRAY, 10);
            if (tag)
                hwgc->sub_state = 10;
        }

        if (hwgc->sub_state == 10)
        {
            scanning_in_young = (heap_region_type & 0x2) != 0;
            uintptr_t low = to_obj + 24 + start * 8;
            uintptr_t high = to_obj + 24 + (start + hwgc->pars.chunkSize) * 8;
            p = to_obj + 24;
            q = p + (start + hwgc->pars.chunkSize) * 8;
            if (p < low)
                p = low;
            if (q > high)
                q = high;

            hwgc->state = STEP_TRACE_PLUS;
            hwgc->sub_state = 0;

            hwgc->previous = STEP_TRACE_PLUS;
            hwgc->previous_sub_state = 0;

            hwgc->done_to = STEP_FETCH;
            hwgc->doneto_sub_state = 0;
        }
    }

    static uintptr_t common_m_value;
    static uintptr_t copy2survivor_region_attr_ptr;
    static uint16_t region_attr;
    if (hwgc->state == STEP_COMMON_OOP)
    {
        if (hwgc->sub_state == 0)
        {
            tag = safeAccessHWAddr(hwgc, task, &from_obj, 8, "read common oop task", false, STEP_COMMON_OOP, 1);
            if (tag)
                hwgc->sub_state = 1;
        }

        if (hwgc->sub_state == 1)
        {
            tag = safeAccessHWAddr(hwgc, from_obj, &common_m_value, 8, "read common oop markvalue", false, STEP_COMMON_OOP, 2);
            if (tag)
                hwgc->sub_state = 2;
        }

        if (hwgc->sub_state == 2)
        {
            if ((common_m_value & 0x3) == 0x3)
            {
                to_obj = common_m_value & ~0x3;
                hwgc->sub_state = 3;
            }
            else
            {
                copy2survivor_region_attr_ptr = hwgc->pars.regionAttrBiasedBase + (from_obj >> hwgc->pars.regionAttrShiftBy) * 2;
                hwgc->state = STEP_Copy2Survivor;
                hwgc->sub_state = 0;
            }
        }

        if (hwgc->sub_state == 3)
        {
            tag = safeAccessHWAddr(hwgc, task, &to_obj, 8, "write task obj", true, STEP_COMMON_OOP, 4);
            if (tag)
                hwgc->sub_state = 4;
        }

        if (hwgc->sub_state == 4)
        {
            if ((task ^ to_obj) >> hwgc->pars.logOfHRGrainBytes == 0)
            {
                hwgc->state = STEP_FETCH;
                hwgc->sub_state = 0;
                return;
            }
            else
            {
                tag = safeAccessHWAddr(hwgc, hwgc->pars.heapRegionBiasedBase + (task >> hwgc->pars.heapRegionShiftBy) * 8, &heap_region, 8, "read task heap region ptr", false, STEP_COMMON_OOP, 5);
                if (tag)
                    hwgc->sub_state = 5;
            }
        }

        if (hwgc->sub_state == 5)
        {
            tag = safeAccessHWAddr(hwgc, heap_region + 0xbc, &heap_region_type, 4, "read task heap region type", false, STEP_COMMON_OOP, 6);
            if (tag)
                hwgc->sub_state = 6;
        }

        if (hwgc->sub_state == 6)
        {
            bool type_is_young = (heap_region_type & 0x2) != 0;
            if (!type_is_young)
            {
                tag = safeAccessHWAddr(hwgc, hwgc->pars.regionAttrBiasedBase + (to_obj >> hwgc->pars.regionAttrShiftBy) * 2, &region_attr, 2, "read new obj region attr", false, STEP_COMMON_OOP, 7);
                if (tag)
                    hwgc->sub_state = 7;
            }
            else
            {
                hwgc->state = STEP_FETCH;
                hwgc->sub_state = 0;
                return;
            }
        }

        if (hwgc->sub_state == 7)
        {
            dest = task;
            hwgc->state = STEP_AOP;
            hwgc->sub_state = 0;
            hwgc->previous = STEP_FETCH;
            hwgc->previous_sub_state = 0;
        }
    }

    static uintptr_t klass_ptr, size;
    static int lh, kid, common_oop_array_length;
    static uint16_t copy2survivor_region_attr, age, dest_attr;
    static uintptr_t dest_attr_ptr, monitor_markWord, from_region;
    static uintptr_t buffer_temp, buffer, region_top, region_end;
    static uint young_index;
    static uintptr_t originValue, new_mark, writeSrcMW;
    if (hwgc->state == STEP_Copy2Survivor)
    {
        if (hwgc->sub_state == 0)
        {
            tag = safeAccessHWAddr(hwgc, from_obj + 8, &klass_ptr, 8, "read klasss ptr", false, STEP_Copy2Survivor, 1);
            if (tag)
                hwgc->sub_state = 1;
        }

        if (hwgc->sub_state == 1)
        {
            tag = safeAccessHWAddr(hwgc, klass_ptr + 8, &originValue, 8, "read lh kid", false, STEP_Copy2Survivor, 2);
            if (tag)
                hwgc->sub_state = 2;
        }

        if (hwgc->sub_state == 2)
        {
            lh = (int)originValue;
            kid = (int)(originValue >> 32);
            if (lh > 0)
            {
                size = lh >> 3;
                hwgc->sub_state = 3;
            }
            else
            {
                tag = safeAccessHWAddr(hwgc, from_obj + 16, &common_oop_array_length, 4, "read common oop array length", false, STEP_Copy2Survivor, 3);
                if (tag)
                    hwgc->sub_state = 3;
            }
        }

        if (hwgc->sub_state == 3)
        {
            if (lh < 0)
            {
                size_t temp = (common_oop_array_length << (uint8_t)lh) + (uint8_t)(lh >> 16);
                size = (size_t)((temp & 0x7) ? (temp >> 3) + 1 : (temp >> 3));
            }
            tag = safeAccessHWAddr(hwgc, copy2survivor_region_attr_ptr, &copy2survivor_region_attr, 2, "read copy2survivor region attr", false, STEP_Copy2Survivor, 4);
            if (tag)
                hwgc->sub_state = 4;
        }

        if (hwgc->sub_state == 4)
        {
            int8_t region_attr_type = (int8_t)(copy2survivor_region_attr >> 8);
            dest_attr_ptr = hwgc->pars.parScanThreadStatePtr + 0x178 + region_attr_type * 2;
            age = 0;
            if (region_attr_type == 0)
            {
                if ((common_m_value & 0x1) == 0x0)
                {
                    uintptr_t temp = (common_m_value & 0x2) ? (common_m_value ^ 0x2) : common_m_value;
                    tag = safeAccessHWAddr(hwgc, temp, &monitor_markWord, 8, "read monitor markword", false, STEP_Copy2Survivor, 5);
                    if (tag)
                        hwgc->sub_state = 5;
                }
                else
                {
                    age = (common_m_value >> 3) & 0x1111;
                    hwgc->sub_state = 5;
                }
            }
            else
            {
                tag = safeAccessHWAddr(hwgc, dest_attr_ptr, &dest_attr, 2, "read dest attr", false, STEP_Copy2Survivor, 6);
                if (tag)
                    hwgc->sub_state = 6;
            }
        }

        if (hwgc->sub_state == 5)
        {
            if ((int8_t)(copy2survivor_region_attr >> 8) == 0 && (common_m_value & 0x1) == 0x0)
                age = (monitor_markWord >> 3) & 0x1111;
            if (age < hwgc->pars.ageThreshold)
            {
                dest_attr_ptr = copy2survivor_region_attr_ptr;
                dest_attr = copy2survivor_region_attr;
                hwgc->sub_state = 6;
            }
            else
            {
                tag = safeAccessHWAddr(hwgc, dest_attr_ptr, &dest_attr, 2, "read dest attr", false, STEP_Copy2Survivor, 6);
                if (tag)
                    hwgc->sub_state = 6;
            }
        }

        if (hwgc->sub_state == 6)
        {
            tag = safeAccessHWAddr(hwgc, hwgc->pars.heapRegionBiasedBase + (from_obj >> hwgc->pars.heapRegionShiftBy) * 8, &from_region, 8, "read from region", false, STEP_Copy2Survivor, 7);
            if (tag)
                hwgc->sub_state = 7;
        }

        if (hwgc->sub_state == 7)
        {
            int8_t dest_attr_type = (int8_t)(dest_attr >> 8);
            tag = safeAccessHWAddr(hwgc, hwgc->pars.plabAllocatorPtr + 0x10 + dest_attr_type * 8, &buffer_temp, 8, "read buffer allocator", false, STEP_Copy2Survivor, 8);
            if (tag)
                hwgc->sub_state = 8;
        }

        if (hwgc->sub_state == 8)
        {
            tag = safeAccessHWAddr(hwgc, buffer_temp, &buffer, 8, "read buffer", false, STEP_Copy2Survivor, 9);
            if (tag)
                hwgc->sub_state = 9;
        }

        if (hwgc->sub_state == 9)
        {
            tag = safeAccessHWAddr(hwgc, buffer + 0x30, &region_top, 8, "read region top", false, STEP_Copy2Survivor, 10);
            if (tag)
                hwgc->sub_state = 10;
        }

        if (hwgc->sub_state == 10)
        {
            tag = safeAccessHWAddr(hwgc, buffer + 0x38, &region_end, 8, "read region end", false, STEP_Copy2Survivor, 11);
            if (tag)
                hwgc->sub_state = 11;
        }

        if (hwgc->sub_state == 11)
        {
            if ((region_end - region_top) / 8 >= size)
            {
                to_obj = region_top;
                region_top = region_top + size * 8;
                tag = safeAccessHWAddr(hwgc, buffer + 0x30, &region_top, 8, "write region top", true, STEP_Copy2Survivor, 12);
                if (tag)
                    hwgc->sub_state = 12;
            }
            else
            {
                to_obj = 0;
                hwgc->state = STEP_DEBUG;
                hwgc->softPars.par0 = dest_attr_ptr;
                hwgc->softPars.par1 = from_obj;
                hwgc->softPars.par2 = size;
                hwgc->softPars.par3 = age;
                hwgc->sub_state = 0;
                hwgc->wake_state = STEP_Copy2Survivor;
                hwgc->wake_sub_state = 12;
                if (qatomic_read(&hwgc->status) & HWGC_STATUS_IRQ)
                {
                    bql_lock();
                    hwgc_raise_irq(hwgc, ALLOC_SLOW_IRQ);
                    bql_unlock();
                }
                printf("do copy to survivor\n");
                return;
            }
        }

        if (hwgc->sub_state == 12)
        {
            if (to_obj == 0)
                to_obj = hwgc->softPars.res;
            writeSrcMW = (to_obj & ~0x3) | 0x3;
            tag = safeAccessHWAddr(hwgc, from_obj, &writeSrcMW, 8, "write src oop markword", true, STEP_Copy2Survivor, 13);
            if (tag)
                hwgc->sub_state = 13;
        }

        if (hwgc->sub_state == 13)
        {
            tag = safeAccessHWAddr(hwgc, from_region + 256, &young_index, 4, "read young index", false, STEP_Copy2Survivor, 14);
            if (tag)
                hwgc->sub_state = 14;
        }

        if (hwgc->sub_state == 14)
        {
            tag = safeAccessHWAddr(hwgc, hwgc->pars.youngWordsBase + young_index * 8, &originValue, 8, "read origin value", false, STEP_Copy2Survivor, 15);
            if (tag)
                hwgc->sub_state = 15;
        }

        if (hwgc->sub_state == 15)
        {
            size_t temp = originValue + size;
            tag = safeAccessHWAddr(hwgc, hwgc->pars.youngWordsBase + young_index * 8, &temp, 8, "write young index value", true, STEP_Copy2Survivor, 16);
            if (tag)
                hwgc->sub_state = 16;
        }

        if (hwgc->sub_state == 16)
        {
            new_mark = common_m_value;
            if ((int8_t)(dest_attr >> 8) == 0 && (common_m_value & 0x1) != 0)
                new_mark = (common_m_value & ~(0x1111 << 3)) | (((age + 1 < 15 ? age + 1 : age) & 0x1111) << 3);
            tag = safeAccessHWAddr(hwgc, to_obj, &new_mark, 8, "write dest markword", true, STEP_Copy2Survivor, 17);
            if (tag)
                hwgc->sub_state = 17;
        }

        if (hwgc->sub_state == 17)
        {
            if ((int8_t)(dest_attr >> 8) == 0 && (common_m_value & 0x1) == 0)
            {
                uintptr_t ptr = (common_m_value & 0x2) ? (common_m_value ^ 0x2) : common_m_value;
                tag = safeAccessHWAddr(hwgc, ptr, &originValue, 8, "read ptr markword", false, STEP_Copy2Survivor, 18);
                if (tag)
                    hwgc->sub_state = 18;
            }
            else
                hwgc->sub_state = 19;
        }

        if (hwgc->sub_state == 18)
        {
            originValue = (originValue & ~(0x1111 << 3)) | (((age + 1 < 15 ? age + 1 : age) & 0x1111) << 3);
            uintptr_t ptr = (common_m_value & 0x2) ? (common_m_value ^ 0x2) : common_m_value;
            tag = safeAccessHWAddr(hwgc, ptr, &originValue, 8, "write ptr markword", true, STEP_Copy2Survivor, 19);
            if (tag)
                hwgc->sub_state = 19;
        }

        if (hwgc->sub_state == 19)
        {
            i = 1;
            hwgc->state = STEP_COPY;
            hwgc->sub_state = 0;
        }

        if (hwgc->sub_state == 20)
        {
            if (kid == 4)
            {
                hwgc->state = STEP_COMMON_OOP;
                hwgc->sub_state = 3;
                return;
            }
            else
            {
                hwgc->state = STEP_TRACE;
                hwgc->sub_state = 0;
            }
        }
    }

    static uintptr_t data;
    if (hwgc->state == STEP_COPY)
    {
        if (hwgc->sub_state == 0)
        {
            if (i >= size)
            {
                hwgc->state = STEP_Copy2Survivor;
                hwgc->sub_state = 20;
            }
            else
            {
                tag = safeAccessHWAddr(hwgc, from_obj + i * 8, &data, 8, "read obj pointer data", false, STEP_COPY, 1);
                if (tag)
                    hwgc->sub_state = 1;
            }
        }

        if (hwgc->sub_state == 1)
        {
            tag = safeAccessHWAddr(hwgc, to_obj + i * 8, &data, 8, "write new obj pointer data", true, STEP_COPY, 2);
            if (tag)
                hwgc->sub_state = 2;
        }

        if (hwgc->sub_state == 2)
        {
            ++i;
            hwgc->sub_state = 0;
        }
    }

    static int end, vtable_len, staticCount;
    static uintptr_t start_map, end_map;
    if (hwgc->state == STEP_TRACE)
    {
        if (hwgc->sub_state == 0)
        {
            scanning_in_young = (int8_t)(dest_attr >> 8) == 0;
            if (lh < 0)
            {
                end = common_oop_array_length % hwgc->pars.chunkSize;
                tag = safeAccessHWAddr(hwgc, to_obj + 16, &end, 4, "write dest array length", true, STEP_TRACE, 1);
                if (tag)
                    hwgc->sub_state = 1;
            }
            else
            {
                tag = safeAccessHWAddr(hwgc, klass_ptr + 160, &vtable_len, 4, "read vtable len", false, STEP_TRACE, 5);
                if (tag)
                    hwgc->sub_state = 5;
            }
        }

        if (hwgc->sub_state == 1)
        {
            if (common_oop_array_length > end)
            {
                tag = safeAccessHWAddr(hwgc, hwgc->pars.taskQueueBottomAddr, &array_localBot, 4, "read task queue bottom to push", false, STEP_TRACE, 2);
                if (tag)
                    hwgc->sub_state = 2;
            }
            else
                hwgc->sub_state = 4;
        }

        if (hwgc->sub_state == 2)
        {
            uintptr_t pushData = from_obj + 0x2;
            tag = safeAccessHWAddr(hwgc, hwgc->pars.taskQueueElemsBase + array_localBot * 8, &pushData, 8, "write taskqueue elems", true, STEP_TRACE, 3);
            if (tag)
                hwgc->sub_state = 3;
        }

        if (hwgc->sub_state == 3)
        {
            array_localBot = (array_localBot + 1) & ((1 << 17) - 1);
            tag = safeAccessHWAddr(hwgc, hwgc->pars.taskQueueBottomAddr, &array_localBot, 4, "write taskqueue bottom addr", true, STEP_TRACE, 4);
            if (tag)
                hwgc->sub_state = 4;
        }

        if (hwgc->sub_state == 4)
        {
            uintptr_t low = to_obj + 24;
            uintptr_t high = to_obj + 24 + end * 8;
            p = to_obj + 24;
            q = p + common_oop_array_length * 8;
            if (p < low)
                p = low;
            if (q > high)
                q = high;

            hwgc->state = STEP_TRACE_PLUS;
            hwgc->sub_state = 0;

            hwgc->previous = STEP_TRACE_PLUS;
            hwgc->previous_sub_state = 0;

            hwgc->done_to = STEP_COMMON_OOP;
            hwgc->doneto_sub_state = 3;
        }

        if (hwgc->sub_state == 5)
        {
            tag = safeAccessHWAddr(hwgc, klass_ptr + 296, &originValue, 8, "read itable len and nonStaticOopMap", false, STEP_TRACE, 6);
            if (tag)
                hwgc->sub_state = 6;
        }

        if (hwgc->sub_state == 6)
        {
            int itable_len = originValue >> 32;
            int nonStaticOopMapSize = (int)originValue;
            start_map = (uintptr_t)((uintptr_t *)(klass_ptr + 464) + vtable_len + itable_len);
            end_map = start_map + nonStaticOopMapSize * 8;
            hwgc->sub_state = 7;
        }

        if (hwgc->sub_state == 7)
        {
            if (start_map < end_map)
            {
                end_map -= 8;
                tag = safeAccessHWAddr(hwgc, end_map, &originValue, 8, "read count and offset", false, STEP_TRACE, 8);
                if (tag)
                    hwgc->sub_state = 8;
            }
            else
                hwgc->sub_state = 9;
        }

        if (hwgc->sub_state == 8)
        {
            int offset = (int)originValue;
            int count = originValue >> 32;
            p = to_obj + offset;
            q = p + count * 8;

            hwgc->state = STEP_TRACE_DEC;
            hwgc->sub_state = 0;

            hwgc->previous = STEP_TRACE_DEC;
            hwgc->previous_sub_state = 0;

            hwgc->done_to = STEP_TRACE;
            hwgc->doneto_sub_state = 7;
        }

        if (hwgc->sub_state == 9)
        {
            if (kid == 2)
            {
                tag = safeAccessHWAddr(hwgc, from_obj + 40, &staticCount, 4, "read static count", false, STEP_TRACE, 10);
                if (tag)
                    hwgc->sub_state = 10;
            }
            else if (kid == 1)
            {
                i = 0;
                hwgc->sub_state = 11;
            }
            else
            {
                hwgc->state = STEP_COMMON_OOP;
                hwgc->sub_state = 3;
            }
        }

        if (hwgc->sub_state == 10)
        {
            p = to_obj + 184;
            q = p + staticCount * 8;

            hwgc->state = STEP_TRACE_PLUS;
            hwgc->sub_state = 0;

            hwgc->previous = STEP_TRACE_PLUS;
            hwgc->previous_sub_state = 0;

            hwgc->done_to = STEP_COMMON_OOP;
            hwgc->doneto_sub_state = 3;
        }

        if (hwgc->sub_state == 11)
        {
            if (i != 3)
            {
                src = i == 1 ? from_obj + 16 : to_obj + 40;
                dest = i == 1 ? to_obj + 16 : from_obj + 40;

                ++i;

                hwgc->state = STEP_DO_OOP_WORK;
                hwgc->sub_state = 0;

                hwgc->previous = STEP_TRACE;
                hwgc->previous_sub_state = 11;
            }
            else if (i == 3)
            {
                i = 0;
                hwgc->state = STEP_COMMON_OOP;
                hwgc->sub_state = 3;
            }
        }
    }

    if (hwgc->state == STEP_TRACE_PLUS)
    {
        if (p < q)
        {
            src = p - to_obj + from_obj;
            dest = p;
            p += 8;
            hwgc->state = STEP_DO_OOP_WORK;
        }
        else
        {
            hwgc->state = hwgc->done_to;
            hwgc->sub_state = hwgc->doneto_sub_state;
        }
    }

    if (hwgc->state == STEP_TRACE_DEC)
    {
        if (p < q)
        {
            q -= 8;
            src = q - to_obj + from_obj;
            dest = q;
            hwgc->state = STEP_DO_OOP_WORK;
        }
        else
        {
            hwgc->state = hwgc->done_to;
            hwgc->sub_state = hwgc->doneto_sub_state;
        }
    }

    static uintptr_t heap_oop;
    static uint region;
    static bool bool_base_value;
    if (hwgc->state == STEP_DO_OOP_WORK)
    {
        if (hwgc->sub_state == 0)
        {
            tag = safeAccessHWAddr(hwgc, src, &heap_oop, 8, "read heap oop", false, STEP_DO_OOP_WORK, 1);
            if (tag)
                hwgc->sub_state = 1;
        }

        if (hwgc->sub_state == 1)
        {
            if (heap_oop == 0)
                hwgc->sub_state = 9;
            else
            {
                uintptr_t region_attr_ptr = hwgc->pars.regionAttrBiasedBase + (heap_oop >> hwgc->pars.regionAttrShiftBy) * 2;
                tag = safeAccessHWAddr(hwgc, region_attr_ptr, &region_attr, 2, "read region attr", false, STEP_DO_OOP_WORK, 2);
                if (tag)
                    hwgc->sub_state = 2;
            }
        }

        if (hwgc->sub_state == 2)
        {
            int8_t region_attr_type = region_attr >> 8;
            if (region_attr_type >= 0)
            {
                tag = safeAccessHWAddr(hwgc, hwgc->pars.taskQueueBottomAddr, &array_localBot, 4, "read taskqueue bottom addr", false, STEP_DO_OOP_WORK, 7);
                if (tag)
                    hwgc->sub_state = 7;
            }
            else if (((dest ^ heap_oop) >> hwgc->pars.logOfHRGrainBytes) != 0)
            {
                if (region_attr_type == -2)
                    hwgc->sub_state = 3;
                else
                    hwgc->sub_state = 6;
            }
            else
                hwgc->sub_state = 9;
        }

        if (hwgc->sub_state == 3)
        {
            region = (heap_oop - ((uintptr_t)hwgc->pars.heapRegionBias << hwgc->pars.heapRegionShiftBy)) >> hwgc->pars.logOfHRGrainBytes;
            tag = safeAccessHWAddr(hwgc, hwgc->pars.humogousReclaimCandidateBoolBase + region, &bool_base_value, 1, "read bool base value", false, STEP_DO_OOP_WORK, 4);
            if (tag)
                hwgc->sub_state = 4;
        }

        if (hwgc->sub_state == 4)
        {
            if (!bool_base_value)
                hwgc->sub_state = 6;
            else
            {
                bool_base_value = false;
                tag = safeAccessHWAddr(hwgc, hwgc->pars.humogousReclaimCandidateBoolBase + region, &bool_base_value, 1, "write bool base value", true, STEP_DO_OOP_WORK, 5);
                if (tag)
                    hwgc->sub_state = 5;
            }
        }

        if (hwgc->sub_state == 5)
        {
            uintptr_t region_attr_dest = hwgc->pars.regionAttrBase + region * 2;
            int8_t dest_value = -1;
            tag = safeAccessHWAddr(hwgc, region_attr_dest + 1, &dest_value, 1, "write dest attr type is notincset", true, STEP_DO_OOP_WORK, 6);
            if (tag)
                hwgc->sub_state = 6;
        }

        if (hwgc->sub_state == 6)
        {
            if (scanning_in_young)
            {
                hwgc->state = hwgc->previous;
                hwgc->sub_state = hwgc->previous_sub_state;
            }
            else
            {
                hwgc->state = STEP_AOP;
                hwgc->sub_state = 0;
            }
        }

        if (hwgc->sub_state == 7)
        {
            tag = safeAccessHWAddr(hwgc, hwgc->pars.taskQueueElemsBase + array_localBot * 8, &dest, 8, "write taskqueue elems", true, STEP_DO_OOP_WORK, 8);
            if (tag)
                hwgc->sub_state = 8;
        }

        if (hwgc->sub_state == 8)
        {
            array_localBot = (array_localBot + 1) & ((1 << 17) - 1);
            tag = safeAccessHWAddr(hwgc, hwgc->pars.taskQueueBottomAddr, &array_localBot, 4, "write taskqueue bottom addr", true, STEP_DO_OOP_WORK, 9);
            if (tag)
                hwgc->sub_state = 9;
        }

        if (hwgc->sub_state == 9)
        {
            hwgc->state = hwgc->previous;
            hwgc->sub_state = hwgc->previous_sub_state;
        }
    }

    static uintptr_t byte_map, byte_map_base, res;
    static size_t card_index, last_index;
    static size_t index;
    if (hwgc->state == STEP_AOP)
    {
        if (hwgc->sub_state == 0)
        {
            if ((region_attr & 0xff) == 0)
            {
                hwgc->state = hwgc->previous;
                hwgc->sub_state = hwgc->previous_sub_state;
                return;
            }
            else
            {
                tag = safeAccessHWAddr(hwgc, hwgc->pars.cardTablePtr + 0x38, &byte_map, 8, "read byte map", false, STEP_AOP, 1);
                if (tag)
                    hwgc->sub_state = 1;
            }
        }

        if (hwgc->sub_state == 1)
        {
            tag = safeAccessHWAddr(hwgc, hwgc->pars.cardTablePtr + 0x40, &byte_map_base, 8, "read byte map base", false, STEP_AOP, 2);
            if (tag)
                hwgc->sub_state = 2;
        }

        if (hwgc->sub_state == 2)
        {
            res = byte_map_base + (dest >> 9);
            card_index = res - byte_map;
            tag = safeAccessHWAddr(hwgc, hwgc->pars.parScanThreadStatePtr + 0x1b0, &last_index, 8, "read last enqueued card index", false, STEP_AOP, 3);
            if (tag)
                hwgc->sub_state = 3;
        }

        if (hwgc->sub_state == 3)
        {
            if (card_index == last_index)
            {
                hwgc->state = hwgc->previous;
                hwgc->sub_state = hwgc->previous_sub_state;
            }
            else
            {
                tag = safeAccessHWAddr(hwgc, hwgc->pars.parScanThreadStatePtr + 0x48, &index, 8, "read queue index", false, STEP_AOP, 4);
                if (tag)
                    hwgc->sub_state = 4;
            }
        }

        if (hwgc->sub_state == 4)
        {
            if (index == 0)
            {
                hwgc->softPars.par0 = res;
                hwgc->state = STEP_DEBUG;
                hwgc->sub_state = 0;
                hwgc->wake_state = STEP_AOP;
                hwgc->wake_sub_state = 7;
                if (qatomic_read(&hwgc->status) & HWGC_STATUS_IRQ)
                {
                    bql_lock();
                    hwgc_raise_irq(hwgc, ENQUEUE_FAILED_IRQ);
                    bql_unlock();
                }
#ifdef DEBUG_ENABLE
                printf("%lx tracing now enter the state %x\n", res, STEP_DEBUG);
#endif
                return;
            }
            else
            {
                tag = safeAccessHWAddr(hwgc, hwgc->pars.parScanThreadStatePtr + 0x58, &buffer, 8, "read queue buffer", false, STEP_AOP, 5);
                if (tag)
                    hwgc->sub_state = 5;
            }
        }

        if (hwgc->sub_state == 5)
        {
            index = index / 8 - 1;
            tag = safeAccessHWAddr(hwgc, buffer + index * 8, &res, 8, "write buffer index entry", true, STEP_AOP, 6);
            if (tag)
                hwgc->sub_state = 6;
        }

        if (hwgc->sub_state == 6)
        {
            index = index * 8;
            tag = safeAccessHWAddr(hwgc, hwgc->pars.parScanThreadStatePtr + 0x48, &index, 8, "write queue index", true, STEP_AOP, 7);
            if (tag)
                hwgc->sub_state = 7;
        }

        if (hwgc->sub_state == 7)
        {
            tag = safeAccessHWAddr(hwgc, hwgc->pars.parScanThreadStatePtr + 0x1b0, &card_index, 8, "write last enqueued card index", true, STEP_AOP, 8);
            if (tag)
                hwgc->sub_state = 8;
        }

        if (hwgc->sub_state == 8)
        {
            hwgc->state = hwgc->previous;
            hwgc->sub_state = hwgc->previous_sub_state;
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

#ifdef DEBUG_ENABLE
        printf("do hwgc work\n");
#endif

        while (1)
        {
            do_hwgc_work(hwgc);
            if (hwgc->state == STEP_DONE)
                break;
            if (hwgc->state == STEP_DEBUG || hwgc->state == STEP_PAGE_FAULT)
            {
                qemu_mutex_lock(&hwgc->thr_mutex);
                while ((qatomic_read(&hwgc->status) & HWGC_STATUS_WAKE) == 0)
                    qemu_cond_wait(&hwgc->thr_cond, &hwgc->thr_mutex);

                if (hwgc->state == STEP_PAGE_FAULT)
                {
#ifdef DEBUG_ENABLE
                    if ((hwgc->softPars.par2 & 0xff) == 0x0)
                        printf("read data %lx\n", hwgc->softPars.res);
                    else
                        printf("write success\n");
#endif
                }
                hwgc->state = hwgc->wake_state;
                hwgc->sub_state = hwgc->wake_sub_state;
                qatomic_and(&hwgc->status, ~HWGC_STATUS_WAKE);
                qemu_mutex_unlock(&hwgc->thr_mutex);
            }
        }

#ifdef DEBUG_ENABLE
        printf("do hwgc end\n");
#endif
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
    k->revision = 0x1;                             // 设备的修订版本号
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