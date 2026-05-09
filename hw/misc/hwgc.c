#include "hwgc.h"
#include "qemu/xxhash.h"
#include <time.h>

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
    struct HWGCCache cache;
    struct HWGCSoftRelated softPars;

    CPUState *cpu;
    HWGCTLBEntry tlb_cache[HWGC_TLB_SIZE];
};

static time_t page_fault_time, start_time, end_time;
static time_t work_begin, work_end;
static uint page_fault, enqueued_irq, alloc_irq, grow_irq;

// 获取当前时间（纳秒）
static inline uint64_t get_time_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
}

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
        start_time = get_time_ns();

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

static inline bool try_access(HWGCState *hwgc, int next, uintptr_t addr, void *buf, size_t sz,
                              const char *msg, bool write, int id)
{
    bool tag = safeAccessHWAddr(hwgc, addr, buf, sz, msg, write, id, next);
    if (!tag)
        return false; // 表示失败，调用者应 return;
    hwgc->sub_state = next;
    return true; // 成功
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
    if (addr >= REG_PAR0 && addr <= REG_PAR22)
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
            offsetof(struct HWGCParameter, compressedOopBase),                // REG_PAR21
            offsetof(struct HWGCParameter, compressedKlassPointerBase),       // REG_PAR22

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

    if (addr == REG_PAR23)
    {
        printf("val %lx \n", val);
        hwgc->pars.compressedOopShift = (uint8_t)(val >> 24);
        hwgc->pars.compressedKlassPointerShift = (uint8_t)(val >> 16);
        hwgc->pars.useCompressedOops = (uint8_t)(val >> 8);
        hwgc->pars.useCompressedKlassPointers = (uint8_t)(val);
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

    int OopSize = hwgc->pars.useCompressedOops ? 4 : 8;
    int ArrayLenOff = hwgc->pars.useCompressedKlassPointers ? 12 : 16;
    int ArrayElementOff = hwgc->pars.useCompressedKlassPointers ? 16 : 24;

    static uintptr_t task;

    if (hwgc->state == STEP_FETCH)
    {
        if (hwgc->sub_state == 0)
        {
            if (hwgc->pars.localBot == 0)
            {
                hwgc->state = STEP_DONE;
                hwgc->sub_state = 0;
#ifdef DEBUG_ENABLE
                printf("The jvm taskqueue has handled over, and now enter the state %x\n", STEP_DONE);
#endif
                return;
            }
            else
                TRY_R(1, hwgc->pars.taskQueueElemsBase + (hwgc->pars.localBot - 1) * 8, &task, 8, "read taskqueue elems", STEP_FETCH);
        }

        if (hwgc->sub_state == 1)
        {
            hwgc->state = STEP_DISPATCH;
            hwgc->sub_state = 0;
            hwgc->pars.localBot = hwgc->pars.localBot - 1;
#ifdef DEBUG_ENABLE
            printf("The task is %lx, and now enter the state %x\n", task, hwgc->state);
#endif
        }
    }

    static uintptr_t src;
    if (hwgc->state == STEP_DISPATCH)
    {
        if ((task & 0x3) != 0x2)
        {
            src = task - (task & 0x3);
            hwgc->state = STEP_OOP;
        }
        else
        {
            src = task - 0x2;
            hwgc->state = STEP_ARRAY;
        }
    }

    static uintptr_t array_mw;
    static uintptr_t dest;
    static int src_length, dest_length;
    static int ncreate, iterator, type;
    static uintptr_t heap_region;
    static bool scanning_in_young;
    static uintptr_t p, q;
    if (hwgc->state == STEP_ARRAY)
    {
        if (hwgc->sub_state == 0)
            TRY_R(1, src, &array_mw, 8, "read array middle word", STEP_ARRAY);

        if (hwgc->sub_state == 1)
        {
            dest = array_mw & ~0x3;
            TRY_R(2, src + ArrayLenOff, &src_length, 4, "read src array length", STEP_ARRAY);
        }

        if (hwgc->sub_state == 2)
            TRY_R(3, dest + ArrayLenOff, &dest_length, 4, "read dest array length", STEP_ARRAY);

        if (hwgc->sub_state == 3)
        {
            int temp = dest_length + hwgc->pars.chunkSize;
            TRY_W(4, dest + ArrayLenOff, &temp, 4, "write dest array length", STEP_ARRAY);
        }

        if (hwgc->sub_state == 4)
        {
            uint task_num = dest_length / hwgc->pars.chunkSize;
            uint remaining_tasks = (src_length - dest_length) / hwgc->pars.chunkSize;
            uint _task_limit = (uint)hwgc->pars.stepperOffset;
            uint _task_fanout = hwgc->pars.stepperOffset >> 32;
            uint max_pending = (_task_fanout - 1) * task_num + 1;
            uint pending = MIN(max_pending, MIN(remaining_tasks, _task_limit));
            ncreate = MIN(_task_fanout, MIN(remaining_tasks, _task_limit + 1) - pending);

            iterator = 0;
            hwgc->sub_state = 5;
        }

        if (hwgc->sub_state == 5)
        {
            if (iterator < ncreate)
            {
                uintptr_t pushData = src + 0x2;
                TRY_W(6, hwgc->pars.taskQueueElemsBase + hwgc->pars.localBot * 8, &pushData, 8, "write taskqueue elems", STEP_ARRAY);
            }
            else
                hwgc->sub_state = 7;
        }

        if (hwgc->sub_state == 6)
        {
            iterator++;
            hwgc->pars.localBot = hwgc->pars.localBot + 1;
            hwgc->sub_state = 5;
        }

        if (hwgc->sub_state == 7)
        {
            int oop_shift = dest >> hwgc->pars.heapRegionShiftBy;
            if (hwgc->cache.array_heap_region_cache != oop_shift)
            {
                uintptr_t heap_region_ptr = hwgc->pars.heapRegionBiasedBase + oop_shift * 8;
                TRY_R(8, heap_region_ptr, &heap_region, 8, "read heap region", STEP_ARRAY);
            }
            else
                hwgc->sub_state = 10;
        }

        if (hwgc->sub_state == 8)
            TRY_R(9, heap_region + 0xbc, &type, 4, "read heap region type", STEP_ARRAY);

        if (hwgc->sub_state == 9)
        {
            hwgc->cache.heap_type_is_young = type != 0;
            TRY_R(10, dest + ArrayLenOff, &type, 4, "read new dest array length", STEP_ARRAY);
        }

        if (hwgc->sub_state == 10)
        {
            scanning_in_young = hwgc->cache.heap_type_is_young;
            uintptr_t low = dest + ArrayElementOff + dest_length * OopSize;
            uintptr_t high = dest + ArrayElementOff + (dest_length + hwgc->pars.chunkSize) * OopSize;
            p = dest + ArrayElementOff;
            q = p + typet * OopSize;
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

    uintptr_t do_src, do_dest;
    if (hwgc->state == STEP_TRACE_PLUS)
    {
        if (p < q)
        {
            do_src = p - dest + src;
            do_dest = p;
            p += OopSize;
            hwgc->state = STEP_DO_OOP_WORK;
        }
        else
        {
            hwgc->state = hwgc->done_to;
            hwgc->sub_state = hwgc->doneto_sub_state;
        }
    }

    static uintptr_t heap_oop;
    static uint region, region_attr;
    static bool bool_base_value;
    if (hwgc->state == STEP_DO_OOP_WORK)
    {
        if (hwgc->sub_state == 0)
            TRY_R(1, do_src, &heap_oop, 8, "read heap oop", STEP_DO_OOP_WORK);

        if (hwgc->sub_state == 1)
        {
            if (hwgc->pars.useCompressedOops)
                heap_oop = (uint)heap_oop;

            if (heap_oop == 0)
                hwgc->sub_state = 9;
            else
            {
                if (hwgc->pars.useCompressedOops)
                    heap_oop = hwgc->pars.compressedOopBase + (heap_oop << hwgc->pars.compressedOopShift);

                int shift_obj = heap_oop >> hwgc->pars.regionAttrShiftBy;
                if (shift_obj != hwgc->cache.obj_shift_cache)
                {
                    uintptr_t region_attr_ptr = hwgc->pars.regionAttrBiasedBase + shift_obj * 2 + 1;
                    tag = safeAccessHWAddr(hwgc, region_attr_ptr, &region_attr, 2, "read region attr", false, STEP_DO_OOP_WORK, 2);
                    if (tag)
                    {
                        hwgc->cache.attr_type_cache = region_attr >> 8;
                        hwgc->sub_state = 2;
                    }
                }
                else
                    hwgc->sub_state = 2;
            }
        }

        if (hwgc->sub_state == 2)
        {
            int8_t region_attr_type = hwgc->cache.attr_type_cache;
            if (region_attr_type >= 0)
            {
                uintptr_t writeValue = hwgc->pars.useCompressedOops ? do_dest + 0x1 : do_dest;
                tag = safeAccessHWAddr(hwgc, hwgc->pars.taskQueueElemsBase + hwgc->cache.localBot * 8, &writeValue, 8, "write taskqueue elems", STEP_DO_OOP_WORK, );
                if (tag)
                    hwgc->sub_state = 7;
            }
            else if (((do_dest ^ heap_oop) >> hwgc->pars.logOfHRGrainBytes) != 0)
            {
                if (region_attr_type == -2)
                    hwgc->sub_state = 3;
                else
                    hwgc->sub_state = 6;
            }
            else
                hwgc->sub_state = 7;
        }

        if (hwgc->sub_state == 3)
        {
            region = (heap_oop - ((uintptr_t)hwgc->pars.heapRegionBias << hwgc->pars.heapRegionShiftBy)) >> hwgc->pars.logOfHRGrainBytes;
            if (region != hwgc->cache.do_oop_region_cache)
            {
                tag = safeAccessHWAddr(hwgc, hwgc->pars.humogousReclaimCandidateBoolBase + region, &bool_base_value, 1, "read bool base value", false, STEP_DO_OOP_WORK, 4);
                if (tag)
                    hwgc->sub_state = 4;
            }
            else
                hwgc->sub_state = 6;
        }

        if (hwgc->sub_state == 4)
        {
            if (!bool_base_value)
            {
                hwgc->sub_state = 6;
                hwgc->cache.do_oop_region_cache = region;
            }
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
                hwgc->sub_state = 7;
            else
            {
                hwgc->state = STEP_AOP;
                hwgc->sub_state = 0;
            }
        }

        if (hwgc->sub_state == 7)
        {
            hwgc->state = hwgc->previous;
            hwgc->sub_state = hwgc->previous_sub_state;
        }
    }

    // 设置成256bit
    // 一次性读取 cardTablePtr + 0x38 cardTablePtr + 0x40
    // 一次性读取 parscanThreadPtr + 0x30 ~ 0x48
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
            index = index / 8;
            tag = safeAccessHWAddr(hwgc, hwgc->pars.parScanThreadStatePtr + 0x58, &buffer, 8, "read queue buffer", false, STEP_AOP, 5);
            if (tag)
                hwgc->sub_state = 5;
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
                    tag = safeAccessHWAddr(hwgc, old_node, &originValue, 8, "write old node", true, STEP_AOP, 10);
                    if (tag)
                        hwgc->sub_state = 10;
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
            tag = safeAccessHWAddr(hwgc, buffer + index * 8, &res, 8, "write buffer index entry", true, STEP_AOP, 7);
            if (tag)
                hwgc->sub_state = 7;
        }

        if (hwgc->sub_state == 7)
        {
            index = index * 8;
            tag = safeAccessHWAddr(hwgc, hwgc->pars.parScanThreadStatePtr + 0x48, &index, 8, "write queue index", true, STEP_AOP, 8);
            if (tag)
                hwgc->sub_state = 8;
        }

        if (hwgc->sub_state == 8)
        {
            tag = safeAccessHWAddr(hwgc, hwgc->pars.parScanThreadStatePtr + 0x1b0, &card_index, 8, "write last enqueued card index", true, STEP_AOP, 9);
            if (tag)
                hwgc->sub_state = 9;
        }

        if (hwgc->sub_state == 9)
        {
            hwgc->state = hwgc->previous;
            hwgc->sub_state = hwgc->previous_sub_state;
            return;
        }

        if (hwgc->sub_state == 10)
        {
            tag = safeAccessHWAddr(hwgc, hwgc->pars.parScanThreadStatePtr + 0x20, &node_allocator_ptr, 8, "read node allocator ptr", false, STEP_AOP, 11);
            if (tag)
                hwgc->sub_state = 11;
        }

        if (hwgc->sub_state == 11)
        {
            new_top = 0;
            // @notice: loongarch and x86 this address is not equal
            //          BufferNode::Allocator free_list and free_count
            tag = safeAccessHWAddr(hwgc, node_allocator_ptr + 0x80, &node, 8, "read node", false, STEP_AOP, 12);
            if (tag)
                hwgc->sub_state = 12;
        }

        if (hwgc->sub_state == 12)
        {
            if (node != 0)
            {
                tag = safeAccessHWAddr(hwgc, node + 0x8, &new_top, 8, "read new top", false, STEP_AOP, 13);
                if (tag)
                    hwgc->sub_state = 13;
            }
            else
                hwgc->sub_state = 13;
        }

        if (hwgc->sub_state == 13)
        {
            tag = safeAccessHWAddr(hwgc, node_allocator_ptr + 0x80, &new_top, 8, "write node", true, STEP_AOP, 14);
            if (tag)
                hwgc->sub_state = 14;
        }

        if (hwgc->sub_state == 14)
        {
            if (node != 0)
            {
                originValue = 0;
                tag = safeAccessHWAddr(hwgc, node + 0x8, &originValue, 8, "write node + 0x8", true, STEP_AOP, 15);
                if (tag)
                    hwgc->sub_state = 15;
            }
            else
            {
                ++enqueued_irq;
                hwgc->softPars.par0 = node_allocator_ptr;
                hwgc->state = STEP_DEBUG;
                hwgc->sub_state = 0;
                hwgc->wake_state = STEP_AOP;
                hwgc->wake_sub_state = 15;
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
        {
            if (hwgc->wake_state == STEP_AOP && hwgc->wake_sub_state == 15 && hwgc->softPars.par0 == node_allocator_ptr)
                node = hwgc->softPars.res;
            buffer = node + 0x10;
            tag = safeAccessHWAddr(hwgc, hwgc->pars.parScanThreadStatePtr + 0x58, &buffer, 8, "write buffer", true, STEP_AOP, 16);
            if (tag)
                hwgc->sub_state = 16;
        }

        if (hwgc->sub_state == 16)
        {
            tag = safeAccessHWAddr(hwgc, node_allocator_ptr, &index, 8, "read new index", false, STEP_AOP, 17);
            if (tag)
                hwgc->sub_state = 17;
        }

        if (hwgc->sub_state == 17)
        {
            originValue = index * 8;
            tag = safeAccessHWAddr(hwgc, hwgc->pars.parScanThreadStatePtr + 0x48, &originValue, 8, "write index", true, STEP_AOP, 18);
            if (tag)
                hwgc->sub_state = 18;
        }

        if (hwgc->sub_state == 18)
        {
            if (old_node == 0)
                hwgc->sub_state = 6;
            else
            {
                tag = safeAccessHWAddr(hwgc, hwgc->pars.parScanThreadStatePtr + 0x40, &originValue, 8, "read buffer list ptr + 0x10", false, STEP_AOP, 19);
                if (tag)
                    hwgc->sub_state = 19;
            }
        }

        if (hwgc->sub_state == 19)
        {
            originValue = originValue + index;
            tag = safeAccessHWAddr(hwgc, hwgc->pars.parScanThreadStatePtr + 0x40, &originValue, 8, "write buffer list ptr + 0x10", true, STEP_AOP, 20);
            if (tag)
                hwgc->sub_state = 20;
        }

        if (hwgc->sub_state == 20)
        {
            tag = safeAccessHWAddr(hwgc, hwgc->pars.parScanThreadStatePtr + 0x30, &originValue, 8, "read buffer list ptr", false, STEP_AOP, 21);
            if (tag)
                hwgc->sub_state = 21;
        }

        if (hwgc->sub_state == 21)
        {
            tag = safeAccessHWAddr(hwgc, old_node + 0x8, &originValue, 8, "write old node + 0x8", true, STEP_AOP, 22);
            if (tag)
                hwgc->sub_state = 22;
        }

        if (hwgc->sub_state == 22)
        {
            tag = safeAccessHWAddr(hwgc, hwgc->pars.parScanThreadStatePtr + 0x30, &old_node, 8, "write buffer list ptr", true, STEP_AOP, 23);
            if (tag)
                hwgc->sub_state = 23;
        }

        if (hwgc->sub_state == 23)
        {
            tag = safeAccessHWAddr(hwgc, hwgc->pars.parScanThreadStatePtr + 0x38, &originValue, 8, "read buffer list ptr + 0x8", false, STEP_AOP, 24);
            if (tag)
                hwgc->sub_state = 24;
        }

        if (hwgc->sub_state == 24)
        {
            if (originValue == 0)
            {
                tag = safeAccessHWAddr(hwgc, hwgc->pars.parScanThreadStatePtr + 0x38, &old_node, 8, "write buffer list ptr + 0x8", true, STEP_AOP, 6);
                if (tag)
                    hwgc->sub_state = 6;
            }
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

        printf("do hwgc work\n");
        page_fault_time = 0;
        page_fault = 0;
        enqueued_irq = 0;
        alloc_irq = 0;
        grow_irq = 0;

        work_begin = get_time_ns();

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

        qatomic_and(&hwgc->status, ~HWGC_STATUS_COMPUTING);
        smp_mb__after_rmw();
        if (qatomic_read(&hwgc->status) & HWGC_STATUS_IRQ)
        {
            bql_lock();
            hwgc_raise_irq(hwgc, COMPLETE_IRQ);
            bql_unlock();
        }

        work_end = get_time_ns();

        printf("do hwgc end(time %.3f), page fault %d, enqueued %d, alloc %d, grow %d\n", (work_end - work_begin) / 1000000.0, page_fault, enqueued_irq, alloc_irq, grow_irq);
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
