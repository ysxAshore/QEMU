#include "hwgc.h"
#include "qemu/xxhash.h"

#define TYPE_PCI_HWGC_DEVICE "hwgc"
typedef struct HWGCState HWGCState;
DECLARE_INSTANCE_CHECKER(HWGCState, HWGC, TYPE_PCI_HWGC_DEVICE)

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

static uint page_fault, enqueue_irq, alloc_irq, grow_irq;

static uint64_t get_device_id(HWGCState *s) { return HWGC_DEVICE_ID; }
static uint64_t get_status(HWGCState *s) { return qatomic_read(&s->status); }
static uint64_t get_int_status(HWGCState *s) { return s->irq_status; }
static uint64_t get_soft_par0(HWGCState *s) { return s->softPars.par0; }
static uint64_t get_soft_par1(HWGCState *s) { return s->softPars.par1; }
static uint64_t get_soft_par2(HWGCState *s) { return s->softPars.par2; }
static uint64_t get_soft_par3(HWGCState *s) { return s->softPars.par3; }

// PCI/PCIe 支持两种中断机制
//      MSI: 边沿出发 不需要维持状态 发一次会自动无效掉
//      Legacy INTx: 电平触发 需要手动拉低 hwgc_lower_irq
// 检查是否启用 msi function
static bool hwgc_msi_enabled(HWGCState *hwgc) { return msi_enabled(&hwgc->pdev); }
static void hwgc_raise_irq(HWGCState *hwgc, uint32_t val)
{
    hwgc->irq_status |= val;
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

static inline unsigned int hwgc_tlb_hash(uintptr_t va_page) { return (va_page >> 12) & 0xffffff; }
static inline void flush_hwgc_tlb(HWGCState *hwgc) { memset(hwgc->tlb_cache, 0, HWGC_TLB_SIZE * sizeof(HWGCTLBEntry)); }
static hwaddr hwgc_translate_va(HWGCState *hwgc, uintptr_t va)
{
    uintptr_t va_page = va & TARGET_PAGE_MASK;
    unsigned int idx = hwgc_tlb_hash(va_page);
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

#ifdef DEBUG_ENABLE
    uint64_t print_value;
    memcpy(&print_value, buf, size);
    printf("%s va %lx access %d(write or read) data (size=%d): 0x%lx\n", debug_info, va, write, size, print_value);
#endif

    return 0;
}
static bool safeAccessHWAddr(HWGCState *hwgc, uintptr_t addr, void *data, int size, const char *debug_info, bool write, enum HWGC_EXEC_STEP prev, int wake_sub_state)
{
    int ret = access_hwaddr(hwgc, addr, data, size, write, debug_info);

    if (ret == -1)
    {
        ++page_fault;
        uint64_t value = 0;
        if (size <= 8 && size > 0)
            memcpy(&value, data, size);

#ifdef DEBUG_ENABLE
        if (write)
            printf("%s va %lx data(size=%d): 0x%lx ---", debug_info, addr, size, value);
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
        if (regs[i].offset == addr)
        {
            if (size != regs[i].width)
                return ~0ULL;
            return regs[i].get(hwgc);
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
    if (addr >= REG_PAR0 && addr <= REG_PAR21)
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
            offsetof(struct HWGCParameter, cardTablePtr),                     // REG_PAR13
            offsetof(struct HWGCParameter, g1h),                              // REG_PAR14
            offsetof(struct HWGCParameter, intArrayKlassObj),                 // REG_PAR15
            offsetof(struct HWGCParameter, objectKlass),                      // REG_PAR16
            offsetof(struct HWGCParameter, lockPtr),                          // REG_PAR17
            offsetof(struct HWGCParameter, thread),                           // REG_PAR18
            offsetof(struct HWGCParameter, dummyRegion),                      // REG_PAR19
            offsetof(struct HWGCParameter, numaPtr),                          // REG_PAR20

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

    if (addr == REG_SOFT_PAR3)
        hwgc->softPars.par3 = val;

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

static void do_hwgc_work(void *opaque)
{
    HWGCState *hwgc = opaque;
    bool tag;

    static uintptr_t task;
    static uint fetch_localBot;
    if (hwgc->state == STEP_FETCH)
    {
        if (hwgc->sub_state == 0)
            TRY_R(1, hwgc->pars.taskQueueBottomAddr, &fetch_localBot, 4, "read taskqueue bottom addr", STEP_FETCH);

        if (hwgc->sub_state == 1)
        {
            if (fetch_localBot == 0)
            {
                hwgc->state = STEP_DONE;
                hwgc->sub_state = 0;
                printf("page fault %d, enqueue irq %d, alloc irq %d, grow irq %d\n", page_fault, enqueue_irq, alloc_irq, grow_irq);
#ifdef DEBUG_ENABLE
                printf("The jvm taskqueue has handled over, and now enter the state %x\n", STEP_DONE);
#endif
                return;
            }
            else
            {
                fetch_localBot = (fetch_localBot - 1) & ((1 << 17) - 1);
                TRY_W(2, hwgc->pars.taskQueueBottomAddr, &fetch_localBot, 4, "write taskqueue bottom", STEP_FETCH);
            }
        }

        if (hwgc->sub_state == 2)
            TRY_R(3, hwgc->pars.taskQueueElemsBase + fetch_localBot * 8, &task, 8, "read taskqueue elems", STEP_FETCH);

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
            TRY_R(1, from_obj, &partial_m_value, 8, "read partial markword", STEP_PARTIAL_ARRAY);
        }

        if (hwgc->sub_state == 1)
        {
            to_obj = partial_m_value & ~0x3;
            TRY_R(2, from_obj + 16, &partial_from_length, 4, "read partial from obj length", STEP_PARTIAL_ARRAY);
        }

        if (hwgc->sub_state == 2)
            TRY_R(3, to_obj + 16, &start, 4, "read partial to obj length", STEP_PARTIAL_ARRAY);

        if (hwgc->sub_state == 3)
        {
            int temp = start + hwgc->pars.chunkSize;
            TRY_W(4, to_obj + 16, &temp, 4, "write partial to obj length", STEP_PARTIAL_ARRAY);
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
                TRY_R(6, hwgc->pars.taskQueueBottomAddr, &array_localBot, 4, "read taskqueue bottom addr", STEP_PARTIAL_ARRAY);
            else
                hwgc->sub_state = 8;
        }

        if (hwgc->sub_state == 6)
        {
            uintptr_t pushData = from_obj + 0x2;
            TRY_W(7, hwgc->pars.taskQueueElemsBase + array_localBot * 8, &pushData, 8, "write taskqueue elems", STEP_PARTIAL_ARRAY);
        }

        if (hwgc->sub_state == 7)
        {
            array_localBot = (array_localBot + 1) & ((1 << 17) - 1);
            ++i;
            TRY_W(5, hwgc->pars.taskQueueBottomAddr, &array_localBot, 4, "write taskqueue bottom addr", STEP_PARTIAL_ARRAY);
        }

        if (hwgc->sub_state == 8)
        {
            uintptr_t heap_region_ptr = hwgc->pars.heapRegionBiasedBase + (to_obj >> hwgc->pars.heapRegionShiftBy) * 8;
            TRY_R(9, heap_region_ptr, &heap_region, 8, "read heap region", STEP_PARTIAL_ARRAY);
        }

        if (hwgc->sub_state == 9)
            TRY_R(10, heap_region + 0xbc, &heap_region_type, 4, "read heap region type", STEP_PARTIAL_ARRAY);

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
            TRY_R(1, task, &from_obj, 8, "read common oop task", STEP_COMMON_OOP);

        if (hwgc->sub_state == 1)
            TRY_R(2, from_obj, &common_m_value, 8, "read common oop markvalue", STEP_COMMON_OOP);

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
            TRY_W(4, task, &to_obj, 8, "write task obj", STEP_COMMON_OOP);

        if (hwgc->sub_state == 4)
        {
            if ((task ^ to_obj) >> hwgc->pars.logOfHRGrainBytes == 0)
            {
                hwgc->state = STEP_FETCH;
                hwgc->sub_state = 0;
                return;
            }
            else
                TRY_R(5, hwgc->pars.heapRegionBiasedBase + (task >> hwgc->pars.heapRegionShiftBy) * 8, &heap_region, 8, "read task heap region ptr", STEP_COMMON_OOP);
        }

        if (hwgc->sub_state == 5)
            TRY_R(6, heap_region + 0xbc, &heap_region_type, 4, "read task heap region type", STEP_COMMON_OOP);

        if (hwgc->sub_state == 6)
        {
            bool type_is_young = (heap_region_type & 0x2) != 0;
            if (!type_is_young)
                TRY_R(7, hwgc->pars.regionAttrBiasedBase + (to_obj >> hwgc->pars.regionAttrShiftBy) * 2, &region_attr, 2, "read new obj region attr", STEP_COMMON_OOP);
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
            TRY_R(1, from_obj + 8, &klass_ptr, 8, "read klasss ptr", STEP_Copy2Survivor);

        if (hwgc->sub_state == 1)
            TRY_R(2, klass_ptr + 8, &originValue, 8, "read lh kid", STEP_Copy2Survivor);

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
                TRY_R(3, from_obj + 16, &common_oop_array_length, 4, "read common oop array length", STEP_Copy2Survivor);
        }

        if (hwgc->sub_state == 3)
        {
            if (lh < 0)
            {
                size_t temp = (common_oop_array_length << (uint8_t)lh) + (uint8_t)(lh >> 16);
                size = (size_t)((temp & 0x7) ? (temp >> 3) + 1 : (temp >> 3));
            }
            TRY_R(4, copy2survivor_region_attr_ptr, &copy2survivor_region_attr, 2, "read copy2survivor region attr", STEP_Copy2Survivor);
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
                    TRY_R(5, temp, &monitor_markWord, 8, "read monitor markword", STEP_Copy2Survivor);
                }
                else
                {
                    age = (common_m_value >> 3) & 0x1111;
                    hwgc->sub_state = 5;
                }
            }
            else
                TRY_R(6, dest_attr_ptr, &dest_attr, 2, "read dest attr", STEP_Copy2Survivor);
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
                TRY_R(6, dest_attr_ptr, &dest_attr, 2, "read dest attr", STEP_Copy2Survivor);
        }

        if (hwgc->sub_state == 6)
            TRY_R(7, hwgc->pars.heapRegionBiasedBase + (from_obj >> hwgc->pars.heapRegionShiftBy) * 8, &from_region, 8, "read from region", STEP_Copy2Survivor);

        if (hwgc->sub_state == 7)
        {
            int8_t dest_attr_type = (int8_t)(dest_attr >> 8);
            TRY_R(8, hwgc->pars.plabAllocatorPtr + 0x10 + dest_attr_type * 8, &buffer_temp, 8, "read buffer allocator", STEP_Copy2Survivor);
        }

        if (hwgc->sub_state == 8)
            TRY_R(9, buffer_temp, &buffer, 8, "read buffer", STEP_Copy2Survivor);

        if (hwgc->sub_state == 9)
            TRY_R(10, buffer + 0x30, &region_top, 8, "read region top", STEP_Copy2Survivor);

        if (hwgc->sub_state == 10)
            TRY_R(11, buffer + 0x38, &region_end, 8, "read region end", STEP_Copy2Survivor);

        if (hwgc->sub_state == 11)
        {
            if ((region_end - region_top) / 8 >= size)
            {
                to_obj = region_top;
                region_top = region_top + size * 8;
                TRY_W(12, buffer + 0x30, &region_top, 8, "write region top", STEP_Copy2Survivor);
            }
            else
            {
                to_obj = 0;
                hwgc->sub_state = 0;
                hwgc->state = STEP_ALLOC;
            }
        }

        if (hwgc->sub_state == 12)
        {
            writeSrcMW = (to_obj & ~0x3) | 0x3;
            TRY_W(13, from_obj, &writeSrcMW, 8, "write src oop markword", STEP_Copy2Survivor);
        }

        if (hwgc->sub_state == 13)
            TRY_R(14, from_region + 256, &young_index, 4, "read young index", STEP_Copy2Survivor);

        if (hwgc->sub_state == 14)
            TRY_R(15, hwgc->pars.youngWordsBase + young_index * 8, &originValue, 8, "read origin value", STEP_Copy2Survivor);

        if (hwgc->sub_state == 15)
        {
            size_t temp = originValue + size;
            TRY_W(16, hwgc->pars.youngWordsBase + young_index * 8, &temp, 8, "write young index value", STEP_Copy2Survivor);
        }

        if (hwgc->sub_state == 16)
        {
            new_mark = common_m_value;
            if ((int8_t)(dest_attr >> 8) == 0 && (common_m_value & 0x1) != 0)
                new_mark = (common_m_value & ~(0x1111 << 3)) | (((age + 1 < 15 ? age + 1 : age) & 0x1111) << 3);
            TRY_W(17, to_obj, &new_mark, 8, "write dest markword", STEP_Copy2Survivor);
        }

        if (hwgc->sub_state == 17)
        {
            if ((int8_t)(dest_attr >> 8) == 0 && (common_m_value & 0x1) == 0)
            {
                uintptr_t ptr = (common_m_value & 0x2) ? (common_m_value ^ 0x2) : common_m_value;
                TRY_R(18, ptr, &originValue, 8, "read ptr markword", STEP_Copy2Survivor);
            }
            else
                hwgc->sub_state = 19;
        }

        if (hwgc->sub_state == 18)
        {
            originValue = (originValue & ~(0x1111 << 3)) | (((age + 1 < 15 ? age + 1 : age) & 0x1111) << 3);
            uintptr_t ptr = (common_m_value & 0x2) ? (common_m_value ^ 0x2) : common_m_value;
            TRY_W(19, ptr, &originValue, 8, "write ptr markword", STEP_Copy2Survivor);
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

    static int8_t dest_attr_type;
    if (hwgc->state == STEP_ALLOC)
    {
        if (hwgc->sub_state == 0)
        {
            dest_attr_type = (int8_t)(dest_attr >> 8);
            hwgc->state = STEP_ALLOCATE_DIRECT;
            hwgc->sub_state = 0;
            hwgc->previous = STEP_ALLOC;
            hwgc->previous_sub_state = 1;
        }

        if (hwgc->sub_state == 1)
        {
#ifdef DEBUG_ENABLE
            printf("new obj is %lx\n", to_obj);
#endif
            if (to_obj == 0)
                TRY_R(2, hwgc->pars.plabAllocatorPtr + 0x18, &buffer_temp, 8, "read old buffer temp ptr", STEP_ALLOC);
            else
                hwgc->sub_state = 7;
        }

        if (hwgc->sub_state == 2)
            TRY_R(3, buffer_temp, &buffer, 8, "read old buffer", STEP_ALLOC);

        if (hwgc->sub_state == 3)
            TRY_R(4, buffer + 0x30, &region_top, 8, "read old region top", STEP_ALLOC);

        if (hwgc->sub_state == 4)
            TRY_R(5, buffer + 0x38, &region_end, 8, "read old region bottom", STEP_ALLOC);

        if (hwgc->sub_state == 5)
        {
            dest_attr_type = 1;
            dest_attr = (dest_attr & 0xff) | ((int16_t)dest_attr_type << 8);
            if ((region_end - region_top) / 8 >= size)
            {
                to_obj = region_top;
                region_top = region_top + size * 8;
                TRY_W(6, buffer + 0x30, &region_top, 8, "write old region top", STEP_ALLOC);
            }
            else
            {
                hwgc->state = STEP_ALLOCATE_DIRECT;
                hwgc->sub_state = 0;
                hwgc->previous = STEP_ALLOC;
                hwgc->previous_sub_state = 6;
            }
        }

        if (hwgc->sub_state == 6)
            TRY_W(7, dest_attr_ptr + 1, &dest_attr_type, 1, "write dest attr ptr", STEP_ALLOC);

        if (hwgc->sub_state == 7)
        {
            hwgc->state = STEP_Copy2Survivor;
            hwgc->sub_state = 12;
        }
    }

    static uintptr_t plab_stats_ptr, allocator_ptr, alloc_klass_ptr;
    static size_t plab_word_size, required_in_plab, actual_plab_size;
    static size_t min_word_size, desired_word_size;
    static int during_gc_select;
    if (hwgc->state == STEP_ALLOCATE_DIRECT)
    {
        if (hwgc->sub_state == 0)
        {
            if (dest_attr_type == 0)
                plab_stats_ptr = hwgc->pars.g1h + 0x250;
            else if (dest_attr_type == 1)
                plab_stats_ptr = hwgc->pars.g1h + 0x2e0;

            TRY_R(1, plab_stats_ptr + 0x30, &originValue, 8, "read plab stats ptr + 0x30", STEP_ALLOCATE_DIRECT);
        }

        if (hwgc->sub_state == 1)
        {
            plab_word_size = MIN(MAX(originValue, 0x102), 0x40000);
            required_in_plab = size + 0x2;

            TRY_R(2, hwgc->pars.plabAllocatorPtr + 0x8, &allocator_ptr, 8, "read allocator ptr", STEP_ALLOCATE_DIRECT);
        }

        if (hwgc->sub_state == 2)
        {
            bool may_throw_away_buffer = required_in_plab * 100 < plab_word_size * 0xa;
            if ((required_in_plab <= plab_word_size) && may_throw_away_buffer)
                TRY_R(3, hwgc->pars.plabAllocatorPtr + 0x10 + dest_attr_type * 8, &buffer_temp, 8, "read buffer temp", STEP_ALLOCATE_DIRECT);
            else
                hwgc->sub_state = 25;
        }

        if (hwgc->sub_state == 3)
            TRY_R(4, buffer_temp, &buffer, 8, "read buffer", STEP_ALLOCATE_DIRECT);

        if (hwgc->sub_state == 4)
            TRY_R(5, buffer + 0x30, &region_top, 8, "read region top", STEP_ALLOCATE_DIRECT);

        if (hwgc->sub_state == 5)
            TRY_R(6, buffer + 0x40, &region_end, 8, "read region end", STEP_ALLOCATE_DIRECT);

        if (hwgc->sub_state == 6)
        {
            if (region_top < region_end)
            {
                size_t words = (region_end - region_top) / 8;
                if (words >= 3)
                {
                    size_t payload_size = words - 3;
                    size_t len = payload_size * 8 / 4;
                    alloc_klass_ptr = hwgc->pars.intArrayKlassObj;
                    TRY_W(7, region_top + 16, &len, 4, "write arraylen", STEP_ALLOCATE_DIRECT);
                }
                else if (words > 0)
                {
                    alloc_klass_ptr = hwgc->pars.objectKlass;
                    hwgc->sub_state = 7;
                }
            }
            else
                hwgc->sub_state = 14;
        }

        if (hwgc->sub_state == 7)
        {
            originValue = 1;
            TRY_W(8, region_top, &originValue, 8, "write region top", STEP_ALLOCATE_DIRECT);
        }

        if (hwgc->sub_state == 8)
            TRY_W(9, region_top + 0x8, &alloc_klass_ptr, 8, "write region top + 0x8", STEP_ALLOCATE_DIRECT);

        if (hwgc->sub_state == 9)
            TRY_W(10, buffer + 0x38, &region_end, 8, "write buffer + 0x38", STEP_ALLOCATE_DIRECT);

        if (hwgc->sub_state == 10)
            TRY_W(11, buffer + 0x30, &region_end, 8, "write buffer + 0x30", STEP_ALLOCATE_DIRECT);

        if (hwgc->sub_state == 11)
            TRY_W(12, buffer + 0x28, &region_end, 8, "write buffer + 0x28", STEP_ALLOCATE_DIRECT);

        if (hwgc->sub_state == 12)
            TRY_R(13, buffer + 0x50, &originValue, 8, "read buffer 0x50", STEP_ALLOCATE_DIRECT);

        if (hwgc->sub_state == 13)
        {
            originValue = originValue + (region_end - region_top) / 8;
            TRY_W(14, buffer + 0x50, &originValue, 8, "write buffer + 0x50", STEP_ALLOCATE_DIRECT);
        }

        if (hwgc->sub_state == 14)
            TRY_R(15, hwgc->pars.plabAllocatorPtr + 0x30 + dest_attr_type * 8, &originValue, 8, "read num plab fills", STEP_ALLOCATE_DIRECT);

        if (hwgc->sub_state == 15)
        {
            originValue = originValue + 1;
            TRY_W(16, hwgc->pars.plabAllocatorPtr + 0x30 + dest_attr_type * 8, &originValue, 8, "write num plab fills", STEP_ALLOCATE_DIRECT);
        }

        if (hwgc->sub_state == 16)
        {
            hwgc->state = STEP_ALLOCATE_DURING_GC;
            hwgc->sub_state = 0;
            min_word_size = required_in_plab;
            desired_word_size = plab_word_size;

            during_gc_select = 0;
        }

        if (hwgc->sub_state == 17)
        {
            if (to_obj != 0)
                TRY_W(18, buffer + 0x20, &actual_plab_size, 8, "write buffer + 0x20", STEP_ALLOCATE_DIRECT);
            else
                hwgc->sub_state = 25;
        }

        if (hwgc->sub_state == 18)
            TRY_W(19, buffer + 0x28, &to_obj, 8, "write buffer + 0x28", STEP_ALLOCATE_DIRECT);

        if (hwgc->sub_state == 19)
        {
            originValue = to_obj + actual_plab_size * 8;
            TRY_W(20, buffer + 0x40, &originValue, 8, "write buffer + 0x40", STEP_ALLOCATE_DIRECT);
        }

        if (hwgc->sub_state == 20)
        {
            originValue = to_obj + (actual_plab_size - 2) * 8;
            TRY_W(21, buffer + 0x38, &originValue, 8, "write buffer + 0x38", STEP_ALLOCATE_DIRECT);
        }

        if (hwgc->sub_state == 21)
            TRY_R(22, buffer + 0x48, &originValue, 8, "read buffer + 0x48", STEP_ALLOCATE_DIRECT);

        if (hwgc->sub_state == 22)
        {
            originValue = originValue + actual_plab_size;
            TRY_W(23, buffer + 0x48, &originValue, 8, "write buffer + 0x48", STEP_ALLOCATE_DIRECT);
        }

        if (hwgc->sub_state == 23)
        {
            size_t delta = actual_plab_size - 0x2;
            if (delta >= size)
                originValue = to_obj + size * 8;
            else
            {
                originValue = to_obj;
                to_obj = 0;
            }
            TRY_W(24, buffer + 0x30, &originValue, 8, "write buffer + 0x30", STEP_ALLOCATE_DIRECT);
        }

        if (hwgc->sub_state == 24)
        {
            hwgc->state = hwgc->previous;
            hwgc->sub_state = hwgc->previous_sub_state;
            return;
        }

        if (hwgc->sub_state == 25)
        {
            hwgc->state = STEP_ALLOCATE_DURING_GC;
            hwgc->sub_state = 0;
            min_word_size = size;
            desired_word_size = size;

            during_gc_select = 1;
        }
    }

    static uintptr_t region_ptr;
    static uintptr_t alloc_region;
    static uintptr_t card_table_ptr, byte_map_base, first, last;
    static int par_alloc_iml_sel;
    static int par_alloc_sel;
    static bool bot_updates;
    if (hwgc->state == STEP_ALLOCATE_DURING_GC)
    {
        if (hwgc->sub_state == 0)
        {
            if (dest_attr_type == 0)
                TRY_R(1, allocator_ptr + 0x28, &region_ptr, 8, "read survivor gc alloc ptr", STEP_ALLOCATE_DURING_GC);
            else
            {
                region_ptr = allocator_ptr + 0x30;
                hwgc->sub_state = 1;
            }
        }

        if (hwgc->sub_state == 1)
            TRY_R(2, region_ptr + 0x8, &alloc_region, 8, "read alloc region", STEP_ALLOCATE_DURING_GC);

        if (hwgc->sub_state == 2)
        {
            if (dest_attr_type == 0)
            {
                par_alloc_iml_sel = 0;
                hwgc->state = STEP_PAR_ALLOCATE_IML;
                hwgc->sub_state = 0;
            }
            else
            {
                par_alloc_sel = 0;
                bot_updates = true;
                hwgc->state = STEP_PAR_ALLOCATE;
                hwgc->sub_state = 0;
            }
        }

        if (hwgc->sub_state == 3)
        {
            if (to_obj == 0)
                TRY_R(9, allocator_ptr + 0x10, &originValue, 1, "read is full value", STEP_ALLOCATE_DURING_GC);
            else
                hwgc->sub_state = 4;
        }

        if (hwgc->sub_state == 4)
        {
            if (to_obj != 0 && dest_attr_type == 0)
                TRY_R(5, hwgc->pars.g1h + 0x78, &card_table_ptr, 8, "read card table ptr", STEP_ALLOCATE_DURING_GC);
            else
                hwgc->sub_state = 8;
        }

        if (hwgc->sub_state == 5)
            TRY_R(6, card_table_ptr + 0x40, &byte_map_base, 8, "read byte map base", STEP_ALLOCATE_DURING_GC);

        if (hwgc->sub_state == 6)
        {
            first = byte_map_base + (to_obj >> 9);
            last = byte_map_base + ((to_obj + actual_plab_size * 8 - 8) >> 9);
            hwgc->sub_state = 7;
        }

        if (hwgc->sub_state == 7)
        {
            if (first < last)
            {
                uintptr_t temp = first;
                originValue = 4;
                ++first;
                TRY_W(7, temp, &originValue, 1, "write first", STEP_ALLOCATE_DURING_GC);
            }
            else
                hwgc->sub_state = 8;
        }

        if (hwgc->sub_state == 8)
        {
            hwgc->sub_state = during_gc_select ? 24 : 17;
            hwgc->state = STEP_ALLOCATE_DIRECT;
            return;
        }

        if (hwgc->sub_state == 9)
        {
            bool is_full = dest_attr_type == 0 ? originValue & 0x1 : originValue & 0x2;
            if (!is_full)
                TRY_W(10, hwgc->pars.lockPtr, &hwgc->pars.thread, 8, "write lock ptr", STEP_ALLOCATE_DURING_GC);
            else
                hwgc->sub_state = 4;
        }

        if (hwgc->sub_state == 10)
        {
            if (dest_attr_type == 0)
            {
                par_alloc_iml_sel = 1;
                hwgc->state = STEP_PAR_ALLOCATE_IML;
                hwgc->sub_state = 0;
            }
            else
            {
                par_alloc_sel = 1;
                bot_updates = true;
                hwgc->state = STEP_PAR_ALLOCATE;
                hwgc->sub_state = 0;
            }
        }

        if (hwgc->sub_state == 11)
        {
            if (to_obj == 0)
            {
                hwgc->state = STEP_ATTEMPT_ALLOC;
                hwgc->sub_state = 0;
            }
            else
                hwgc->sub_state = 13;
        }

        if (hwgc->sub_state == 12)
        {
            if (to_obj == 0)
            {
                uintptr_t addr = dest_attr_type == 0 ? allocator_ptr + 0x10 : allocator_ptr + 0x11;
                originValue = 1;
                TRY_W(13, addr, &originValue, 1, "write allocator_ptr + 0x10/11", STEP_ALLOCATE_DURING_GC);
            }
            else
                hwgc->sub_state = 13;
        }

        if (hwgc->sub_state == 13)
        {
            originValue = 0;
            TRY_W(4, hwgc->pars.lockPtr, &originValue, 8, "write lock ptr", STEP_ALLOCATE_DURING_GC);
        }
    }

    static uintptr_t alloc_top, alloc_end;
    static size_t want_to_allocate;
    if (hwgc->state == STEP_PAR_ALLOCATE_IML)
    {
        if (hwgc->sub_state == 0)
            TRY_R(1, alloc_region + 0x10, &alloc_top, 8, "read alloc region + 0x10", STEP_PAR_ALLOCATE_IML);

        if (hwgc->sub_state == 1)
            TRY_R(2, alloc_region + 0x8, &alloc_end, 8, "read alloc region + 0x8", STEP_PAR_ALLOCATE_IML);

        if (hwgc->sub_state == 2)
        {
            size_t available = (alloc_end - alloc_top) / 8;
            want_to_allocate = available > desired_word_size ? desired_word_size : available;
            if (want_to_allocate >= min_word_size)
            {
                actual_plab_size = want_to_allocate;
                originValue = alloc_top + want_to_allocate * 8;
                to_obj = alloc_top;
                TRY_W(3, alloc_region + 0x10, &originValue, 8, "write alloc region + 0x10", STEP_PAR_ALLOCATE_IML);
            }
            else
            {
                to_obj = 0;
                hwgc->sub_state = 3;
            }
        }

        if (hwgc->sub_state == 3)
        {
            hwgc->sub_state = par_alloc_iml_sel == 2 ? 1 : (par_alloc_iml_sel == 1 ? 11 : 3);
            hwgc->state = par_alloc_iml_sel == 2 ? STEP_PAR_ALLOCATE : STEP_ALLOCATE_DURING_GC;
        }
    }

    static uintptr_t blk_start, blk_end, bot_part_ptr, bot_ptr;
    static uintptr_t next_offset_threshold, array, reserved_start, begin;
    static size_t index, start_card_for_region, start_card, end_card, reach, num_cards;
    static uint8_t ct_offset;
    if (hwgc->state == STEP_PAR_ALLOCATE)
    {
        if (hwgc->sub_state == 0)
        {
            par_alloc_iml_sel = 2;
            hwgc->state = STEP_PAR_ALLOCATE_IML;
            hwgc->sub_state = 0;
            return;
        }

        if (hwgc->sub_state == 1)
        {
            if (to_obj != 0)
            {
                blk_start = to_obj;
                blk_end = to_obj + actual_plab_size * 8;
                bot_part_ptr = alloc_region + 0x20;
                TRY_R(2, bot_part_ptr, &next_offset_threshold, 8, "read next offset threshold", STEP_PAR_ALLOCATE);
            }
            else
                hwgc->sub_state = 13;
        }

        if (hwgc->sub_state == 2)
        {
            if (blk_end > next_offset_threshold)
                TRY_R(3, bot_part_ptr + 0x8, &index, 8, "read botpart index", STEP_PAR_ALLOCATE);
            else
                hwgc->sub_state = 13;
        }

        if (hwgc->sub_state == 3)
            TRY_R(4, bot_part_ptr + 0x10, &bot_ptr, 8, "read bot ptr", STEP_PAR_ALLOCATE);

        if (hwgc->sub_state == 4)
            TRY_R(5, bot_ptr + 0x10, &array, 8, "read bot array", STEP_PAR_ALLOCATE);

        if (hwgc->sub_state == 5)
        {
            originValue = (next_offset_threshold - blk_start) / 8;
            TRY_W(6, array + index, &originValue, 1, "write array index", STEP_PAR_ALLOCATE);
        }

        if (hwgc->sub_state == 6)
            TRY_R(7, bot_ptr, &reserved_start, 8, "read bot ptr", STEP_PAR_ALLOCATE);

        if (hwgc->sub_state == 7)
        {
            size_t end_index = (blk_end - 8 - reserved_start) >> 9;
            uintptr_t rem_st = reserved_start + ((index + 1) << 6) * 8;
            uintptr_t rem_end = reserved_start + ((end_index << 6) + 64) * 8;
            start_card = (rem_st - reserved_start) >> 9;
            end_card = (rem_end - 8 - reserved_start) >> 9;
            if (index + 1 <= end_index && rem_st < rem_end && start_card <= end_card)
            {
                start_card_for_region = start_card;
                ct_offset = 0xff;
                i = 0;
                hwgc->sub_state = 8;
            }
            else
                hwgc->sub_state = 11;
        }

        if (hwgc->sub_state == 8)
        {
            if (i < 14)
            {
                reach = start_card - 1 + ((1 << (4 * (i + 1))) - 1);
                ct_offset = 64 + i;
                num_cards = (reach >= end_card ? end_card : reach) - start_card_for_region + 1;
                begin = array + start_card_for_region;
                ++i;
                hwgc->sub_state = 9;
            }
            else
                hwgc->sub_state = 11;
        }

        if (hwgc->sub_state == 9)
        {
            if (num_cards > 0)
            {
                originValue = begin;
                ++begin;
                --num_cards;
                TRY_W(9, originValue, &ct_offset, 1, "write begin", STEP_PAR_ALLOCATE);
            }
            else
                hwgc->sub_state = 10;
        }

        if (hwgc->sub_state == 10)
        {
            start_card_for_region = reach + 1;
            if (reach >= end_card)
                hwgc->sub_state = 11;
            else
                hwgc->sub_state = 8;
        }

        if (hwgc->sub_state == 11)
        {
            size_t end_index = (blk_end - 8 - reserved_start) >> 9;
            index = end_index + 1;
            next_offset_threshold = reserved_start + ((end_index << 6) + 64) * 8;
            TRY_W(12, bot_part_ptr, &next_offset_threshold, 8, "write next offset threshold", STEP_PAR_ALLOCATE);
        }

        if (hwgc->sub_state == 12)
            TRY_W(13, bot_part_ptr + 0x8, &index, 8, "write bot part index", STEP_PAR_ALLOCATE);

        if (hwgc->sub_state == 13)
        {
            hwgc->sub_state = par_alloc_sel == 2 ? 17 : (par_alloc_sel == 1 ? 11 : 3);
            hwgc->state = par_alloc_sel == 2 ? STEP_ATTEMPT_ALLOC : STEP_ALLOCATE_DURING_GC;
        }
    }

    static size_t allocated_bytes;
    static int8_t type;
    static uintptr_t new_alloc_region;
    if (hwgc->state == STEP_ATTEMPT_ALLOC)
    {
        if (hwgc->sub_state == 0)
        {
            if (alloc_region != hwgc->pars.dummyRegion)
                TRY_R(1, alloc_region, &alloc_end, 8, "read alloc region", STEP_ATTEMPT_ALLOC);
            else
                hwgc->sub_state = 2;
        }

        if (hwgc->sub_state == 1)
            TRY_R(2, alloc_region + 0x10, &alloc_top, 8, "read alloc region + 0x10", STEP_ATTEMPT_ALLOC);

        if (hwgc->sub_state == 2)
            TRY_R(3, region_ptr + 0x18, &originValue, 8, "read region ptr + 0x18", STEP_ATTEMPT_ALLOC);

        if (hwgc->sub_state == 3)
        {
            allocated_bytes = alloc_top - alloc_end - originValue;
            TRY_R(4, hwgc->pars.g1h + 0x240, &originValue, 8, "read g1h + 0x240", STEP_ATTEMPT_ALLOC);
        }

        if (hwgc->sub_state == 4)
        {
            originValue = originValue + allocated_bytes;
            TRY_W(5, hwgc->pars.g1h + 0x240, &originValue, 8, "write g1h + 0x240", STEP_ATTEMPT_ALLOC);
        }

        if (hwgc->sub_state == 5)
            TRY_R(6, region_ptr + 0x40, &type, 1, "read purpose attr", STEP_ATTEMPT_ALLOC);

        if (hwgc->sub_state == 6)
        {
            uintptr_t ptr = type == 1 ? hwgc->pars.g1h + 0xa0 : hwgc->pars.g1h + 0x3f8;
            TRY_R(7, ptr, &originValue, 8, "read oldset or survivor", STEP_ATTEMPT_ALLOC);
        }

        if (hwgc->sub_state == 7)
        {
            uintptr_t ptr = hwgc->pars.g1h + (type == 1 ? 0xa0 : 0x3f8);
            originValue = originValue + (type == 1 ? 1 : allocated_bytes);
            TRY_W(8, ptr, &originValue, 8, "write oldset or survivor", STEP_ATTEMPT_ALLOC);
        }

        if (hwgc->sub_state == 8)
        {
            originValue = 0;
            TRY_W(9, region_ptr + 0x18, &originValue, 8, "write used byte before", STEP_ATTEMPT_ALLOC);
        }

        if (hwgc->sub_state == 9)
            TRY_W(10, region_ptr + 0x8, &hwgc->pars.dummyRegion, 8, "write alloc region", STEP_ATTEMPT_ALLOC);

        if (hwgc->sub_state == 10)
        {
            // printf("needs allocate\n");
            // hwgc->state = STEP_DEBUG;
            // hwgc->sub_state = 0;

            // hwgc->softPars.par0 = region_ptr;
            // hwgc->softPars.par1 = desired_word_size;

            // hwgc->wake_state = STEP_ATTEMPT_ALLOC;
            // hwgc->wake_sub_state = 11;
            // if (qatomic_read(&hwgc->status) & HWGC_STATUS_IRQ)
            //{
            //     bql_lock();
            //     hwgc_raise_irq(hwgc, DEBUG_IRQ);
            //     bql_unlock();
            // }
            // return;
            hwgc->state = STEP_NEW_GC_ALLOC;
            hwgc->sub_state = 0;
        }

        if (hwgc->sub_state == 11)
        {
            // new_alloc_region = hwgc->softPars.res;
            if (new_alloc_region != 0)
            {
                originValue = 0;
                TRY_W(12, new_alloc_region + 0xa8, &originValue, 8, "write pre dummy top", STEP_ATTEMPT_ALLOC);
            }
            else
                hwgc->sub_state = 20;
        }

        if (hwgc->sub_state == 12)
            TRY_R(13, new_alloc_region, &alloc_end, 8, "read new alloc region", STEP_ATTEMPT_ALLOC);

        if (hwgc->sub_state == 13)
            TRY_R(14, new_alloc_region + 0x10, &alloc_top, 8, "read new alloc region + 0x10", STEP_ATTEMPT_ALLOC);

        if (hwgc->sub_state == 14)
        {
            originValue = alloc_top - alloc_end;
            TRY_W(15, region_ptr + 0x18, &originValue, 8, "write region ptr + 0x18", STEP_ATTEMPT_ALLOC);
        }

        if (hwgc->sub_state == 15)
            TRY_R(16, region_ptr + 0x20, &bot_updates, 1, "read bot updates", STEP_ATTEMPT_ALLOC);

        if (hwgc->sub_state == 16)
        {
            alloc_region = new_alloc_region;
            min_word_size = desired_word_size;

            par_alloc_sel = 2;
            hwgc->state = STEP_PAR_ALLOCATE;
            hwgc->sub_state = 0;
            return;
        }

        if (hwgc->sub_state == 17)
            TRY_W(18, region_ptr + 0x8, &new_alloc_region, 8, "write region ptr + 0x8", STEP_ATTEMPT_ALLOC);

        if (hwgc->sub_state == 18)
            TRY_R(19, region_ptr + 0x10, &originValue, 4, "read region ptr + 0x10", STEP_ATTEMPT_ALLOC);

        if (hwgc->sub_state == 19)
        {
            originValue = originValue + 1;
            TRY_W(20, region_ptr + 0x10, &originValue, 4, "write region ptr + 0x10", STEP_ATTEMPT_ALLOC);
        }

        if (hwgc->sub_state == 20)
        {
            if (new_alloc_region != 0 && to_obj != 0)
                actual_plab_size = desired_word_size;
            else
                to_obj = 0;
            hwgc->state = STEP_ALLOCATE_DURING_GC;
            hwgc->sub_state = 12;
            return;
        }
    }

    static uint region_node_index, array_len, array_max;
    static uintptr_t policy_ptr, grow_array_ptr, data_ptr, count_per_node, numa;
    static bool expand_failure, allocate_free_sel;
    if (hwgc->state == STEP_NEW_GC_ALLOC)
    {
        if (hwgc->sub_state == 0)
            TRY_R(1, region_ptr + 0x30, &region_node_index, 4, "read region node index", STEP_NEW_GC_ALLOC);

        if (hwgc->sub_state == 1)
            TRY_R(2, hwgc->pars.g1h + 0x3f8 + 0x8, &grow_array_ptr, 8, "read grow array ptr", STEP_NEW_GC_ALLOC);

        if (hwgc->sub_state == 2)
            TRY_R(3, hwgc->pars.g1h + 0x430, &policy_ptr, 8, "read policy ptr", STEP_NEW_GC_ALLOC);

        if (hwgc->sub_state == 3)
        {
            heap_region_type = type == 1 ? 0x10 : 0x3;

            allocate_free_sel = false;
            hwgc->state = STEP_ALLOC_FREE_REGION;
            hwgc->sub_state = 0;
            // printf("needs allocate\n");
            // hwgc->state = STEP_DEBUG;
            // hwgc->sub_state = 0;

            // hwgc->softPars.par0 = desired_word_size;
            // hwgc->softPars.par1 = heap_region_type;
            // hwgc->softPars.par2 = region_node_index;

            // hwgc->wake_state = STEP_NEW_GC_ALLOC;
            // hwgc->wake_sub_state = 4;
            // if (qatomic_read(&hwgc->status) & HWGC_STATUS_IRQ)
            //{
            //     bql_lock();
            //     hwgc_raise_irq(hwgc, DEBUG_IRQ);
            //     bql_unlock();
            // }
            // return;
        }

        if (hwgc->sub_state == 22)
            TRY_R(23, hwgc->pars.g1h + 0x370, &expand_failure, 1, "read expand faiulre", STEP_NEW_GC_ALLOC);

        if (hwgc->sub_state == 23)
        {
            if (new_alloc_region == 0 && (expand_failure & 0xff))
            {
                hwgc->state = STEP_DEBUG;
                hwgc->sub_state = 0;

                hwgc->softPars.par0 = region_node_index;

                hwgc->wake_state = STEP_NEW_GC_ALLOC;
                hwgc->wake_sub_state = 24;
                if (qatomic_read(&hwgc->status) & HWGC_STATUS_IRQ)
                {
                    bql_lock();
                    hwgc_raise_irq(hwgc, DEBUG_IRQ);
                    bql_unlock();
                }
                return;
            }
            else
                hwgc->sub_state = 4;
        }

        if (hwgc->sub_state == 24)
        {
            tag = hwgc->softPars.res & 0xff;
            originValue = 0;
            if (tag)
            {
                allocate_free_sel = true;
                hwgc->state = STEP_ALLOC_FREE_REGION;
                hwgc->sub_state = 0;
            }
            else
                TRY_W(4, hwgc->pars.g1h + 0x370, &originValue, 1, "write expand faiulre", STEP_NEW_GC_ALLOC);
        }

        if (hwgc->sub_state == 4)
        {
            // new_alloc_region = hwgc->softPars.res;
            if (new_alloc_region != 0)
                TRY_W(5, new_alloc_region + 0xbc, &heap_region_type, 4, "write heap reion type", STEP_NEW_GC_ALLOC);
            else
                hwgc->sub_state = 21;
        }

        if (hwgc->sub_state == 5)
        {
            if (heap_region_type == 0x3)
                TRY_R(6, grow_array_ptr, &originValue, 8, "read grow array len and max", STEP_NEW_GC_ALLOC);
            else
                hwgc->sub_state = 16;
        }

        if (hwgc->sub_state == 6)
        {
            array_max = originValue >> 32;
            array_len = (uint)originValue;
            if (array_len == array_max)
            {
                hwgc->state = STEP_DEBUG;
                hwgc->sub_state = 0;

                hwgc->softPars.par0 = grow_array_ptr;
                hwgc->softPars.par1 = array_len;

                hwgc->wake_state = STEP_NEW_GC_ALLOC;
                hwgc->wake_sub_state = 7;
                if (qatomic_read(&hwgc->status) & HWGC_STATUS_IRQ)
                {
                    bql_lock();
                    hwgc_raise_irq(hwgc, ALLOC_SLOW_IRQ);
                    bql_unlock();
                }
                return;
            }
            else
                hwgc->sub_state = 7;
        }

        if (hwgc->sub_state == 7)
        {
            originValue = array_len + 1;
            TRY_W(8, grow_array_ptr, &originValue, 4, "write grow array len", STEP_NEW_GC_ALLOC);
        }

        if (hwgc->sub_state == 8)
            TRY_R(9, grow_array_ptr + 0x8, &data_ptr, 8, "read grow array data ptr", STEP_NEW_GC_ALLOC);

        if (hwgc->sub_state == 9)
            TRY_W(10, data_ptr + array_len * 8, &new_alloc_region, 8, "write data ptr", STEP_NEW_GC_ALLOC);

        if (hwgc->sub_state == 10)
            TRY_R(11, hwgc->pars.g1h + 0x3f8 + 0x18, &count_per_node, 8, "read count per node ptr", STEP_NEW_GC_ALLOC);

        if (hwgc->sub_state == 11)
            TRY_R(12, hwgc->pars.g1h + 0x3f8 + 0x20, &numa, 8, "read numa", STEP_NEW_GC_ALLOC);

        if (hwgc->sub_state == 12)
            TRY_R(13, numa + 0x18, &originValue, 4, "read active node ids", STEP_NEW_GC_ALLOC);

        if (hwgc->sub_state == 13)
            TRY_R(14, new_alloc_region + 0x120, &array_len, 4, "read new node index", STEP_NEW_GC_ALLOC);

        if (hwgc->sub_state == 14)
        {
            if (array_len < originValue)
                TRY_R(15, count_per_node + array_len * 4, &originValue, 4, "read count per node + new node index * 4", STEP_NEW_GC_ALLOC);
            else
                hwgc->sub_state = 16;
        }

        if (hwgc->sub_state == 15)
        {
            originValue = originValue + 1;
            TRY_W(16, count_per_node + array_len * 4, &originValue, 4, "write count per node + new node index * 4", STEP_NEW_GC_ALLOC);
        }

        if (hwgc->sub_state == 16)
            TRY_R(17, new_alloc_region + 0xb0, &count_per_node, 8, "read remset ptr", STEP_NEW_GC_ALLOC);

        if (hwgc->sub_state == 17)
            TRY_R(18, new_alloc_region + 0xb8, &originValue, 8, "read hrm index and type", STEP_NEW_GC_ALLOC);

        if (hwgc->sub_state == 18)
        {
            array_len = originValue >> 32;
            array_max = (uint)originValue;
            originValue = (array_len & 0x2) != 0 ? 2 : ((array_len & 0x10) != 0 ? 0 : 1);
            if (originValue != 1)
                TRY_W(19, count_per_node + 0xf0, &originValue, 4, "write state ptr", STEP_NEW_GC_ALLOC);
            else
                hwgc->sub_state = 19;
        }

        if (hwgc->sub_state == 19)
            TRY_R(20, hwgc->pars.g1h + 0x580 + 0x10, &count_per_node, 8, "read region attr base", STEP_NEW_GC_ALLOC);

        if (hwgc->sub_state == 20)
        {
            bool needs_remset_update = (array_len & 0x10) == 0;
            TRY_W(21, count_per_node + array_max * 2, &needs_remset_update, 1, "write region attr base + hrm index * 2", STEP_NEW_GC_ALLOC);
        }

        if (hwgc->sub_state == 21)
        {
            hwgc->sub_state = 11;
            hwgc->state = STEP_ATTEMPT_ALLOC;
            return;
        }
    }

    static bool from_head;
    static uint active_node_ids, region_size, page_size, cur_depth, max_depth;
    static uintptr_t free_list_ptr, cur, prev, next;
    if (hwgc->state == STEP_ALLOC_FREE_REGION)
    {
        if (hwgc->sub_state == 0)
            TRY_R(1, hwgc->pars.numaPtr + 0x18, &active_node_ids, 4, "read active node ids", STEP_ALLOC_FREE_REGION);

        if (hwgc->sub_state == 1)
        {
            new_alloc_region = 0;
            free_list_ptr = hwgc->pars.g1h + 0x130 + 0xb0;
            from_head = (heap_region_type & 0x2) == 0;
            if (region_node_index != UINT_MAX - 1 && active_node_ids > 1)
                TRY_R(2, hwgc->pars.numaPtr + 0x20, &region_size, 4, "read region size", STEP_ALLOC_FREE_REGION);
            else
                hwgc->sub_state = 12;
        }

        if (hwgc->sub_state == 2)
            TRY_R(3, hwgc->pars.numaPtr + 0x28, &page_size, 4, "read page size", STEP_ALLOC_FREE_REGION);

        if (hwgc->sub_state == 3)
        {
            cur_depth = 0;
            max_depth = 3 * MAX((uint)(page_size / region_size), 1u) * active_node_ids;
            uintptr_t addr = free_list_ptr + (from_head ? 0x28 : 0x30);
            TRY_R(4, addr, &cur, 8, "read cur", STEP_ALLOC_FREE_REGION);
        }

        if (hwgc->sub_state == 4)
        {
            if (cur != 0 && cur_depth < max_depth)
                TRY_R(5, cur + 0x120, &originValue, 4, "read node index", STEP_ALLOC_FREE_REGION);
            else
                hwgc->sub_state = 6;
        }

        if (hwgc->sub_state == 5)
        {
            if (region_node_index == (uint)originValue)
                hwgc->sub_state = 6;
            else
            {
                ++cur_depth;
                uintptr_t addr = cur + (from_head ? 0xd0 : 0xd8);
                TRY_R(4, addr, &cur, 8, "read next cur", STEP_ALLOC_FREE_REGION);
            }
        }

        if (hwgc->sub_state == 6)
        {
            if (cur == 0 || cur_depth >= max_depth)
            {
                new_alloc_region = 0;
                hwgc->sub_state = 12;
            }
            else
            {
                new_alloc_region = cur;
                TRY_R(7, new_alloc_region + 0xd8, &prev, 8, "read prev", STEP_ALLOC_FREE_REGION);
            }
        }

        if (hwgc->sub_state == 7)
            TRY_R(8, new_alloc_region + 0xd0, &next, 8, "read next", STEP_ALLOC_FREE_REGION);

        if (hwgc->sub_state == 8)
        {
            uintptr_t addr = prev == 0 ? free_list_ptr + 0x28 : prev + 0xd0;
            TRY_W(9, addr, &next, 8, "write value next", STEP_ALLOC_FREE_REGION);
        }

        if (hwgc->sub_state == 9)
        {
            uintptr_t addr = next == 0 ? free_list_ptr + 0x30 : prev + 0xd8;
            TRY_W(10, addr, &prev, 8, "write value prev", STEP_ALLOC_FREE_REGION);
        }

        if (hwgc->sub_state == 10)
        {
            originValue = 0;
            TRY_W(11, new_alloc_region + 0xd0, &originValue, 8, "write next region", STEP_ALLOC_FREE_REGION);
        }

        if (hwgc->sub_state == 11)
            TRY_W(12, new_alloc_region + 0xd8, &originValue, 8, "write prev region", STEP_ALLOC_FREE_REGION);

        if (hwgc->sub_state == 12)
            TRY_R(13, free_list_ptr + 0x10, &active_node_ids, 4, "read length", STEP_ALLOC_FREE_REGION);

        if (hwgc->sub_state == 13)
        {
            if (new_alloc_region == 0)
            {
                if (active_node_ids == 0)
                {
                    new_alloc_region = 0;
                    hwgc->sub_state = 18;
                }
                else
                {
                    uintptr_t addr = free_list_ptr + (from_head ? 0x28 : 0x30);
                    TRY_R(14, addr, &new_alloc_region, 8, "read new alloc region", STEP_ALLOC_FREE_REGION);
                }
            }
            else
                hwgc->sub_state = 18;
        }

        if (hwgc->sub_state == 14)
        {
            uintptr_t addr = new_alloc_region + (from_head ? 0xd0 : 0xd8);
            TRY_R(15, addr, &originValue, 8, "read region next or prev", STEP_ALLOC_FREE_REGION);
        }

        if (hwgc->sub_state == 15)
        {
            uintptr_t addr = free_list_ptr + (from_head ? 0x28 : 0x30);
            TRY_W(16, addr, &originValue, 8, "write region next or prev", STEP_ALLOC_FREE_REGION);
        }

        if (hwgc->sub_state == 16)
        {
            uintptr_t addr;
            if (from_head)
                addr = originValue == 0 ? free_list_ptr + 0x30 : originValue + 0xd8;
            else
                addr = originValue == 0 ? free_list_ptr + 0x28 : originValue + 0xd0;
            originValue = 0;
            TRY_W(17, addr, &originValue, 8, "write 0", STEP_ALLOC_FREE_REGION);
        }

        if (hwgc->sub_state == 17)
        {
            uintptr_t addr = new_alloc_region + (from_head ? 0xd0 : 0xd8);
            TRY_W(18, addr, &originValue, 8, "write region next or prev", STEP_ALLOC_FREE_REGION);
        }

        if (hwgc->sub_state == 18)
        {
            if (new_alloc_region != 0)
                TRY_R(19, free_list_ptr + 0x38, &originValue, 8, "read last ptr", STEP_ALLOC_FREE_REGION);
            else
                hwgc->sub_state = 21;
        }

        if (hwgc->sub_state == 19)
        {
            if (originValue == new_alloc_region)
            {
                originValue = 0;
                TRY_W(20, free_list_ptr + 0x38, &originValue, 8, "write last ptr", STEP_ALLOC_FREE_REGION);
            }
            else
                hwgc->sub_state = 20;
        }

        if (hwgc->sub_state == 20)
        {
            active_node_ids -= 1;
            TRY_W(21, free_list_ptr + 0x10, &active_node_ids, 4, "write length", STEP_ALLOC_FREE_REGION);
        }

        if (hwgc->sub_state == 21)
        {
            hwgc->sub_state = allocate_free_sel ? 4 : 22;
            hwgc->state = STEP_NEW_GC_ALLOC;
            return;
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
                TRY_R(1, from_obj + i * 8, &data, 8, "read obj pointer data", STEP_COPY);
        }

        if (hwgc->sub_state == 1)
            TRY_W(2, to_obj + i * 8, &data, 8, "write new obj pointer data", STEP_COPY);

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
                TRY_W(1, to_obj + 16, &end, 4, "write dest array length", STEP_TRACE);
            }
            else
                TRY_R(5, klass_ptr + 160, &vtable_len, 4, "read vtable len", STEP_TRACE);
        }

        if (hwgc->sub_state == 1)
        {
            if (common_oop_array_length > end)
                TRY_R(2, hwgc->pars.taskQueueBottomAddr, &array_localBot, 4, "read task queue bottom to push", STEP_TRACE);
            else
                hwgc->sub_state = 4;
        }

        if (hwgc->sub_state == 2)
        {
            uintptr_t pushData = from_obj + 0x2;
            TRY_W(3, hwgc->pars.taskQueueElemsBase + array_localBot * 8, &pushData, 8, "write taskqueue elems", STEP_TRACE);
        }

        if (hwgc->sub_state == 3)
        {
            array_localBot = (array_localBot + 1) & ((1 << 17) - 1);
            TRY_W(4, hwgc->pars.taskQueueBottomAddr, &array_localBot, 4, "write taskqueue bottom addr", STEP_TRACE);
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
            TRY_R(6, klass_ptr + 296, &originValue, 8, "read itable len and nonStaticOopMap", STEP_TRACE);

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
                TRY_R(8, end_map, &originValue, 8, "read count and offset", STEP_TRACE);
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
                TRY_R(10, from_obj + 40, &staticCount, 4, "read static count", STEP_TRACE);
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
            TRY_R(1, src, &heap_oop, 8, "read heap oop", STEP_DO_OOP_WORK);

        if (hwgc->sub_state == 1)
        {
            if (heap_oop == 0)
                hwgc->sub_state = 9;
            else
            {
                uintptr_t region_attr_ptr = hwgc->pars.regionAttrBiasedBase + (heap_oop >> hwgc->pars.regionAttrShiftBy) * 2;
                TRY_R(2, region_attr_ptr, &region_attr, 2, "read region attr", STEP_DO_OOP_WORK);
            }
        }

        if (hwgc->sub_state == 2)
        {
            int8_t region_attr_type = region_attr >> 8;
            if (region_attr_type >= 0)
                TRY_R(7, hwgc->pars.taskQueueBottomAddr, &array_localBot, 4, "read taskqueue bottom addr", STEP_DO_OOP_WORK);
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
            TRY_R(4, hwgc->pars.humogousReclaimCandidateBoolBase + region, &bool_base_value, 1, "read bool base value", STEP_DO_OOP_WORK);
        }

        if (hwgc->sub_state == 4)
        {
            if (!bool_base_value)
                hwgc->sub_state = 6;
            else
            {
                bool_base_value = false;
                TRY_W(5, hwgc->pars.humogousReclaimCandidateBoolBase + region, &bool_base_value, 1, "write bool base value", STEP_DO_OOP_WORK);
            }
        }

        if (hwgc->sub_state == 5)
        {
            uintptr_t region_attr_dest = hwgc->pars.regionAttrBase + region * 2;
            int8_t dest_value = -1;
            TRY_W(6, region_attr_dest + 1, &dest_value, 1, "write dest attr type is notincset", STEP_DO_OOP_WORK);
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
            TRY_W(8, hwgc->pars.taskQueueElemsBase + array_localBot * 8, &dest, 8, "write taskqueue elems", STEP_DO_OOP_WORK);

        if (hwgc->sub_state == 8)
        {
            array_localBot = (array_localBot + 1) & ((1 << 17) - 1);
            TRY_W(9, hwgc->pars.taskQueueBottomAddr, &array_localBot, 4, "write taskqueue bottom addr", STEP_DO_OOP_WORK);
        }

        if (hwgc->sub_state == 9)
        {
            hwgc->state = hwgc->previous;
            hwgc->sub_state = hwgc->previous_sub_state;
        }
    }

    static uintptr_t byte_map, res;
    static size_t card_index, last_index;
    static uintptr_t node_allocator_ptr, node, old_node, new_top;
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
                TRY_R(1, hwgc->pars.cardTablePtr + 0x38, &byte_map, 8, "read byte map", STEP_AOP);
        }

        if (hwgc->sub_state == 1)
            TRY_R(2, hwgc->pars.cardTablePtr + 0x40, &byte_map_base, 8, "read byte map base", STEP_AOP);

        if (hwgc->sub_state == 2)
        {
            res = byte_map_base + (dest >> 9);
            card_index = res - byte_map;
            TRY_R(3, hwgc->pars.parScanThreadStatePtr + 0x1b0, &last_index, 8, "read last enqueued card index", STEP_AOP);
        }

        if (hwgc->sub_state == 3)
        {
            if (card_index == last_index)
            {
                hwgc->state = hwgc->previous;
                hwgc->sub_state = hwgc->previous_sub_state;
            }
            else
                TRY_R(4, hwgc->pars.parScanThreadStatePtr + 0x48, &index, 8, "read queue index", STEP_AOP);
        }

        if (hwgc->sub_state == 4)
        {
            index = index / 8;
            TRY_R(5, hwgc->pars.parScanThreadStatePtr + 0x58, &buffer, 8, "read queue buffer", STEP_AOP);
        }

        if (hwgc->sub_state == 5)
        {
            if (index == 0)
            {
                old_node = 0;
                if (buffer != 0)
                {
                    originValue = 0;
                    old_node = buffer - 0x10;
                    TRY_W(10, old_node, &originValue, 8, "write old node", STEP_AOP);
                }
                else
                    hwgc->sub_state = 10;
            }
            else
                hwgc->sub_state = 6;
        }

        if (hwgc->sub_state == 6)
        {
            index = index - 1;
            TRY_W(7, buffer + index * 8, &res, 8, "write buffer index entry", STEP_AOP);
        }

        if (hwgc->sub_state == 7)
        {
            index = index * 8;
            TRY_W(8, hwgc->pars.parScanThreadStatePtr + 0x48, &index, 8, "write queue index", STEP_AOP);
        }

        if (hwgc->sub_state == 8)
            TRY_W(9, hwgc->pars.parScanThreadStatePtr + 0x1b0, &card_index, 8, "write last enqueued card index", STEP_AOP);

        if (hwgc->sub_state == 9)
        {
            hwgc->state = hwgc->previous;
            hwgc->sub_state = hwgc->previous_sub_state;
            return;
        }

        if (hwgc->sub_state == 10)
            TRY_R(11, hwgc->pars.parScanThreadStatePtr + 0x20, &node_allocator_ptr, 8, "read node allocator ptr", STEP_AOP);

        if (hwgc->sub_state == 11)
        {
            new_top = 0;
            // @notice: loongarch and x86 this address is not equal
            //          BufferNode::Allocator free_list and free_count
            TRY_R(12, node_allocator_ptr + 0x80, &node, 8, "read node", STEP_AOP);
        }

        if (hwgc->sub_state == 12)
        {
            if (node != 0)
                TRY_R(13, node + 0x8, &new_top, 8, "read new top", STEP_AOP);
            else
                hwgc->sub_state = 13;
        }

        if (hwgc->sub_state == 13)
            TRY_W(14, node_allocator_ptr + 0x80, &new_top, 8, "write node", STEP_AOP);

        if (hwgc->sub_state == 14)
        {
            if (node != 0)
            {
                originValue = 0;
                TRY_W(15, node + 0x8, &originValue, 8, "write node + 0x8", STEP_AOP);
            }
            else
            {
                hwgc->softPars.par0 = node_allocator_ptr;
                hwgc->state = STEP_DEBUG;
                hwgc->sub_state = 0;
                hwgc->wake_state = STEP_AOP;
                hwgc->wake_sub_state = 17;
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
        }

        if (hwgc->sub_state == 15)
            TRY_R(16, node_allocator_ptr + 0x100, &originValue, 8, "read node allocator ptr + 0x100", STEP_AOP);

        if (hwgc->sub_state == 16)
        {
            originValue = originValue - 1;
            TRY_W(17, node_allocator_ptr + 0x100, &originValue, 8, "write node allocator ptr + 0x100", STEP_AOP);
        }

        if (hwgc->sub_state == 17)
        {
            if (hwgc->wake_state == STEP_AOP && hwgc->wake_sub_state == 17 && hwgc->softPars.par0 == node_allocator_ptr)
                node = hwgc->softPars.res;
            buffer = node + 0x10;
            TRY_W(18, hwgc->pars.parScanThreadStatePtr + 0x58, &buffer, 8, "write buffer", STEP_AOP);
        }

        if (hwgc->sub_state == 18)
            TRY_R(19, node_allocator_ptr, &index, 8, "read new index", STEP_AOP);

        if (hwgc->sub_state == 19)
        {
            originValue = index * 8;
            TRY_W(20, hwgc->pars.parScanThreadStatePtr + 0x48, &originValue, 8, "write index", STEP_AOP);
        }

        if (hwgc->sub_state == 20)
        {
            if (old_node == 0)
                hwgc->sub_state = 6;
            else
                TRY_R(21, hwgc->pars.parScanThreadStatePtr + 0x40, &originValue, 8, "read buffer list ptr + 0x10", STEP_AOP);
        }

        if (hwgc->sub_state == 21)
        {
            originValue = originValue + index;
            TRY_W(22, hwgc->pars.parScanThreadStatePtr + 0x40, &originValue, 8, "write buffer list ptr + 0x10", STEP_AOP);
        }

        if (hwgc->sub_state == 22)
            TRY_R(23, hwgc->pars.parScanThreadStatePtr + 0x30, &originValue, 8, "read buffer list ptr", STEP_AOP);

        if (hwgc->sub_state == 23)
            TRY_W(24, old_node + 0x8, &originValue, 8, "write old node + 0x8", STEP_AOP);

        if (hwgc->sub_state == 24)
            TRY_W(25, hwgc->pars.parScanThreadStatePtr + 0x30, &old_node, 8, "write buffer list ptr", STEP_AOP);

        if (hwgc->sub_state == 25)
            TRY_R(26, hwgc->pars.parScanThreadStatePtr + 0x38, &originValue, 8, "read buffer list ptr + 0x8", STEP_AOP);

        if (hwgc->sub_state == 26)
        {
            if (originValue == 0)
                TRY_W(6, hwgc->pars.parScanThreadStatePtr + 0x38, &old_node, 8, "write buffer list ptr + 0x8", STEP_AOP);
            else
                hwgc->sub_state = 6;
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
