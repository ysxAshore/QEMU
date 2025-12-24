#include "hwgc.h"

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
    bool stop;

    // reg data
    uint32_t status;
    uint32_t irq_status;

    bool cont;
    struct HWGCParameter pars;
    struct HWGCSoftCallParameter softPars;
    uint64_t soft_res;
    enum HWGC_EXEC_STEP current;
    enum HWGC_EXEC_STEP previous;
    enum HWGC_EXEC_STEP done_to;
};

static uint64_t get_device_id(HWGCState *s) { return HWGC_DEVICE_ID; }
static uint64_t get_status(HWGCState *s) { return qatomic_read(&s->status); }
static uint64_t get_int_status(HWGCState *s) { return s->irq_status; }
static uint64_t get_soft_par0(HWGCState *s) { return s->softPars.par0; }
static uint64_t get_soft_par1(HWGCState *s) { return s->softPars.par1; }
static uint64_t get_soft_par2(HWGCState *s) { return s->softPars.par2; }
static uint64_t get_soft_par3(HWGCState *s) { return s->softPars.par3; }

#define VADDR_CACHE_SIZE 4096
typedef struct
{
    uintptr_t va;
    hwaddr pa;
} VAddrCacheEntry;

static VAddrCacheEntry vaddr_cache[VADDR_CACHE_SIZE] = {0};

static inline uint vpage_hash(uintptr_t vpage)
{
    // 简单哈希，可替换为更好的
    return (unsigned int)(vpage ^ (vpage >> 12)) % VADDR_CACHE_SIZE;
}

static hwaddr vaddr2hwaddr(uintptr_t vaddr)
{
    uintptr_t vpage = vaddr & TARGET_PAGE_MASK;
    uintptr_t offset = vaddr & ~TARGET_PAGE_MASK;

    // search cache
    uint index = vpage_hash(vpage);
    if (vaddr_cache[index].va == vpage && vaddr_cache[index].pa != (hwaddr)0)
    {
        printf("cache hit va %lx pa %lx\n", vaddr, vaddr_cache[index].pa | offset);
        return vaddr_cache[index].pa | offset;
    }

    CPUState *cpu = qemu_get_cpu(0);
    hwaddr hpage = cpu_get_phys_page_debug(cpu, vpage);
    if (hpage != (hwaddr)-1)
    {
        vaddr_cache[index].va = vpage;
        vaddr_cache[index].pa = hpage;
        printf("translate success va %lx pa %lx\n", vaddr, hpage | offset);
    }
    return hpage | offset;
}
static int readHWAddr(uintptr_t va, void *value, int size, const char *debug_info)
{
    hwaddr ha = vaddr2hwaddr(va);
    if (ha == (hwaddr)-1)
    {
        printf("%s %lx vaddr to hwaddr is failed\n", debug_info, va);
        return -1;
    }

    uint8_t buf[8] = {0};
    MemTxResult res = address_space_read(&address_space_memory, ha, MEMTXATTRS_UNSPECIFIED, buf, size);
    // if (res != MEMTX_OK)
    //{
    //     printf("address space read failed\n");
    //     return -1;
    // }
    memcpy(value, buf, size);

    // debug
    uint64_t print_value;
    memcpy(&print_value, buf, size);

    printf("%s va %lx read data (size=%d): 0x%lx\n", debug_info, va, size, print_value);
    return 0;
}
static int writeHWAddr(uintptr_t va, void *value, int size, const char *debug_info)
{
    hwaddr ha = vaddr2hwaddr(va);
    if (ha == (hwaddr)-1)
    {
        printf("%s %lx vaddr to hwaddr is failed\n", debug_info, va);
        return -1;
    }

    uint8_t buf[8] = {0};
    memcpy(buf, value, size);
    MemTxResult res = address_space_write(&address_space_memory, ha, MEMTXATTRS_UNSPECIFIED, buf, size);
    // if (res != MEMTX_OK)
    //{
    //     printf("address space write failed\n");
    //     return -1;
    // }

    // debug
    uint64_t print_value;
    memcpy(&print_value, buf, size);
    printf("%s va %lx wrote data (size=%d): 0x%lx\n", debug_info, va, size, print_value);
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
        {
            printf("raise msi\n");
            msi_notify(&hwgc->pdev, 0);
        }
        else
        {
            printf("raise legacy intx\n");
            pci_set_irq(&hwgc->pdev, 1);
        }
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
    if (addr >= REG_PAR0 && addr <= REG_PAR14)
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
            offsetof(struct HWGCParameter, taskQueueAgeTopAddr),              // REG_PAR11
            offsetof(struct HWGCParameter, taskQueueElemsBase),               // REG_PAR12
            offsetof(struct HWGCParameter, humogousReclaimCandidateBoolBase), // REG_PAR13
            offsetof(struct HWGCParameter, cardTablePtr)                      // REG_PAR14
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
        hwgc->current = STEP_FETCH;
        qatomic_or(&hwgc->status, HWGC_STATUS_COMPUTING);
        qemu_cond_signal(&hwgc->thr_cond);
    }

    if (addr == REG_CONTINUE_WORK)
    {
        hwgc->cont = true;
        qemu_cond_signal(&hwgc->thr_cond);
    }

    if (addr == REG_SOFT_RES)
        hwgc->soft_res = val;

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

static void pushTask(void *opaque, uintptr_t pushData)
{
    HWGCState *hwgc = opaque;

    uint localBot;
    readHWAddr(hwgc->pars.taskQueueBottomAddr, &localBot, 4, "read taskqueue bottom addr");
    writeHWAddr(hwgc->pars.taskQueueElemsBase + localBot * 8, &pushData, 8, "write taskqueue elems");
    localBot = (localBot + 1) & ((1 << 17) - 1);
    writeHWAddr(hwgc->pars.taskQueueBottomAddr, &localBot, 4, "write taskqueue bottom addr");
}

// struct TaskParameter
//{
//     uintptr_t task;
//     uint fetch_localBot;
//
// } task_par;

static void do_hwgc_work(void *opaque)
{

    HWGCState *hwgc = opaque;

    static uintptr_t task = 0;
    if (hwgc->current == STEP_FETCH)
    {
        uint localBot;
        readHWAddr(hwgc->pars.taskQueueBottomAddr, &localBot, 4, "read taskqueue bottom addr");
        if (localBot == 0)
        {
            hwgc->current = STEP_DONE;
            printf("The jvm taskqueue has handled over, and now enter the state %x\n", STEP_DONE);
        }
        else
        {
            localBot = (localBot - 1) & ((1 << 17) - 1);
            writeHWAddr(hwgc->pars.taskQueueBottomAddr, &localBot, 4, "write taskqueue bottom");
            readHWAddr(hwgc->pars.taskQueueElemsBase + localBot * 8, &task, 8, "read taskqueue elems");
            printf("The task is %lx, and now enter the state %x\n", task, STEP_DISPATCH);
            hwgc->current = STEP_DISPATCH;
        }
    }

    if (hwgc->current == STEP_DISPATCH)
    {
        if ((task & 0x3) == 0x0)
        {
            hwgc->current = STEP_COMMON_OOP;
            printf("The task is common oop ptr, now enter the state %x\n", STEP_COMMON_OOP);
        }
        else
        {
            hwgc->current = STEP_PARTIAL_ARRAY;
            printf("The task is partial array oop, now enter the state %x\n", STEP_PARTIAL_ARRAY);
        }
    }

    static uintptr_t p = 0, q = 0;
    static uintptr_t from_obj = 0, to_obj = 0;
    static bool scanning_in_young = false;
    if (hwgc->current == STEP_PARTIAL_ARRAY)
    {
        from_obj = task - 0x2;
        uintptr_t m_value;
        readHWAddr(from_obj, &m_value, 8, "read markWord");
        to_obj = m_value & ~0x3;

        int array_length, start;
        readHWAddr(from_obj + 16, &array_length, 4, "read array length");
        readHWAddr(to_obj + 16, &start, 4, "read to_obj length");
        int to_obj_length = start + hwgc->pars.chunkSize;
        writeHWAddr(to_obj + 16, &to_obj_length, 4, "write to_obj length");

        uint32_t task_num = start / hwgc->pars.chunkSize;
        uint32_t remaining_tasks = (array_length - start) / hwgc->pars.chunkSize;
        uint32_t _task_limit = (uint32_t)hwgc->pars.stepperOffset;
        uint32_t _task_fanout = (uint32_t)(hwgc->pars.stepperOffset >> 32);
        uint32_t max_pending = (_task_fanout - 1) * task_num + 1;
        uint32_t pending = MIN(MIN(max_pending, remaining_tasks), _task_limit);
        uint32_t ncreate = MIN(_task_fanout, MIN(remaining_tasks, _task_limit + 1) - pending);

        for (uint32_t i = 0; i < ncreate; ++i)
            pushTask(hwgc, from_obj + 0x2);

        uintptr_t heap_region_ptr = hwgc->pars.heapRegionBiasedBase + (to_obj >> hwgc->pars.heapRegionShiftBy) * 8;
        uintptr_t heap_region;
        readHWAddr(heap_region_ptr, &heap_region, 8, "read heap region");
        int heap_region_type;
        readHWAddr(heap_region + 0xbc, &heap_region_type, 4, "read heap region type");
        scanning_in_young = heap_region_type != 0;

        uintptr_t low = to_obj + 24 + (uint)start * 8;
        uintptr_t high = to_obj + 24 + to_obj_length * 8;
        p = to_obj + 24;
        q = p + to_obj_length * 8;
        if (p < low)
            p = low;
        if (q > high)
            q = high;
        hwgc->current = STEP_TRACE_PLUS;
        hwgc->done_to = STEP_FETCH;
        printf("p is %lx, q is %lx, now enter the state %x\n", p, q, STEP_TRACE_PLUS);
    }

    static uintptr_t src = 0;
    static uintptr_t dest = 0;
    if (hwgc->current == STEP_TRACE_PLUS)
    {
        if (p < q)
        {
            src = p - to_obj + from_obj;
            dest = p;
            p += 8;
            hwgc->current = STEP_DO_OOP_WORK;
            hwgc->previous = STEP_TRACE_PLUS;
            printf("p is %lx, q is %lx, now enter the state %x\n", p, q, STEP_DO_OOP_WORK);
        }
        else
        {
            hwgc->current = hwgc->done_to;
            printf("p >= q, now enter the state %x\n", hwgc->done_to);
        }
    }

    static uint16_t region_attr = 0;
    static uintptr_t region_attr_ptr = 0;
    if (hwgc->current == STEP_DO_OOP_WORK)
    {
        uintptr_t heap_oop;
        readHWAddr(src, &heap_oop, 8, "read heap oop");
        if (heap_oop == 0)
            hwgc->current = hwgc->previous;
        else
        {

            region_attr_ptr = hwgc->pars.regionAttrBiasedBase + (heap_oop >> hwgc->pars.regionAttrShiftBy) * 2;
            readHWAddr(region_attr_ptr, &region_attr, 2, "read region attr type");
            if ((int8_t)(region_attr >> 8) >= 0)
            {
                pushTask(hwgc, dest);
                hwgc->current = hwgc->previous;
            }
            else if (((dest ^ heap_oop) >> hwgc->pars.logOfHRGrainBytes) != 0)
            {
                if ((int8_t)(region_attr >> 8) == -2)
                {
                    size_t pointer_delta = heap_oop - ((uintptr_t)hwgc->pars.heapRegionBias << hwgc->pars.heapRegionShiftBy);
                    uint32_t region = pointer_delta >> hwgc->pars.logOfHRGrainBytes;
                    int8_t value;
                    readHWAddr(hwgc->pars.humogousReclaimCandidateBoolBase + region, &value, 1, "read bool base");
                    if (value)
                    {
                        value = 0;
                        writeHWAddr(hwgc->pars.humogousReclaimCandidateBoolBase + region, &value, 1, "write bool base");
                        uintptr_t region_base_ptr = hwgc->pars.regionAttrBase + region * 2;
                        value = -1;
                        writeHWAddr(region_base_ptr + 1, &value, 1, "write region attr type");
                    }
                }
                if (scanning_in_young == 1)
                    hwgc->current = hwgc->previous;
                else
                    hwgc->current = STEP_AOP;
            }
        }
    }

    static size_t card_index = 0;
    if (hwgc->current == STEP_AOP)
    {
        if ((region_attr & 0xff) == 0)
            hwgc->current = hwgc->previous;
        else
        {
            uintptr_t byte_map, byte_map_base;
            readHWAddr(hwgc->pars.cardTablePtr + 0x38, &byte_map, 8, "read byte map");
            readHWAddr(hwgc->pars.cardTablePtr + 0x40, &byte_map_base, 8, "read byte map base");
            uintptr_t res = byte_map_base + (dest >> 9);
            card_index = res - byte_map;
            size_t ref_card_index;
            readHWAddr(hwgc->pars.parScanThreadStatePtr + 0x1b0, &ref_card_index, 8, "read card index");
            if (ref_card_index != card_index)
            {
                uintptr_t rdc_local_qset_ptr = hwgc->pars.parScanThreadStatePtr + 0x18;
                uintptr_t queue_ptr = rdc_local_qset_ptr + 0x30;
                size_t index;
                readHWAddr(queue_ptr, &index, 8, "read queue index");
                index = index / 8;
                if (index == 0)
                {
                    hwgc->softPars.par0 = res;
                    hwgc->current = STEP_ENQUEUED_INT;
                    if (qatomic_read(&hwgc->status) & HWGC_STATUS_IRQ)
                    {
                        bql_lock();
                        hwgc_raise_irq(hwgc, ENQUEUE_FAILED_IRQ);
                        bql_unlock();
                    }
                }
                else
                {
                    uintptr_t buffer;
                    readHWAddr(queue_ptr + 0x10, &buffer, 8, "read queue buffer");
                    --index;
                    writeHWAddr(buffer + index * 8, &res, 8, "write queue buffer");
                    index = index * 8;
                    writeHWAddr(queue_ptr, &index, 8, "write queue index");
                    hwgc->current = STEP_UPDATE_CARD;
                }
            }
            else
                hwgc->current = hwgc->previous;
        }
    }

    if (hwgc->current == STEP_UPDATE_CARD)
    {
        writeHWAddr(hwgc->pars.parScanThreadStatePtr + 0x1b0, &card_index, 8, "write card index");
        hwgc->current = hwgc->previous;
    }

    static uintptr_t obj = 0;
    static uint64_t m_value = 0;
    if (hwgc->current == STEP_COMMON_OOP)
    {
        readHWAddr(task, &obj, 8, "read common oop");
        region_attr_ptr = hwgc->pars.regionAttrBiasedBase + (obj >> hwgc->pars.regionAttrShiftBy) * 2;
        readHWAddr(region_attr_ptr, &region_attr, 2, "read region attr");
        if ((int8_t)(region_attr >> 8) < 0)
            hwgc->current = STEP_FETCH;
        else
        {

            readHWAddr(obj, &m_value, 8, "read mark word");
            if ((m_value & 0x3) == 0x3)
            {
                obj = m_value & ~0x3;
                hwgc->current = STEP_UPDATE_REF;
            }
            else
                hwgc->current = STEP_Copy2Survivor;
        }
    }

    if (hwgc->current == STEP_UPDATE_REF)
    {
        writeHWAddr(task, &obj, 8, "write common oop");
        if (((task ^ obj) >> hwgc->pars.logOfHRGrainBytes) == 0)
            hwgc->current = STEP_FETCH;
        else
        {
            uintptr_t heap_region_ptr = hwgc->pars.heapRegionBiasedBase + (task >> hwgc->pars.heapRegionShiftBy) * 8;
            uintptr_t heap_region;
            readHWAddr(heap_region_ptr, &heap_region, 8, "read heap region");
            uint heap_region_type;
            readHWAddr(heap_region + 0xbc, &heap_region_type, 4, "read heap region type");
            bool typeIsYoung = (heap_region_type & 0x2) != 0;
            if (!typeIsYoung)
            {
                region_attr_ptr = hwgc->pars.regionAttrBiasedBase + (obj >> hwgc->pars.regionAttrShiftBy) * 2;
                readHWAddr(region_attr_ptr, &region_attr, 2, "read region attr");
                hwgc->current = STEP_AOP;
                hwgc->previous = STEP_FETCH;
            }
            else
                hwgc->current = STEP_FETCH;
        }
    }

    static int lh = 0;
    static int kid = 0;
    static size_t size = 0;
    static uint age = 0;
    static uintptr_t obj_ptr = 0;
    static uintptr_t klass_ptr = 0;
    static uintptr_t from_region = 0;
    static int8_t dest_attr_type = 0;
    if (hwgc->current == STEP_Copy2Survivor)
    {
        uint64_t lh_kid;
        readHWAddr(obj + 0x8, &klass_ptr, 8, "read klass ptr");
        readHWAddr(klass_ptr + 0x8, &lh_kid, 8, "read lh kid");
        lh = (int)lh_kid;
        kid = lh_kid >> 32;
        if (lh > 0)
            size = lh >> 3;
        else
        {
            int array_length;
            readHWAddr(obj + 16, &array_length, 4, "read array length");
            size_t size_in_bytes = (array_length << (uint8_t)lh) + (uint8_t)(lh >> 16);
            size = (size_t)(size_in_bytes & 0x7 ? (size_in_bytes >> 3) + 1 : size_in_bytes >> 3);
        }

        int8_t region_attr_type = (int8_t)(region_attr >> 8);
        uintptr_t dest_attr_ptr = hwgc->pars.parScanThreadStatePtr + 0x178 + region_attr_type * 2;
        if (region_attr_type == 0)
        {
            if ((m_value & 0x1) == 0)
            {
                bool has_monitor = m_value & 0x2;
                uint64_t ptr = has_monitor ? m_value ^ 0x2 : m_value;
                uint64_t mark;
                readHWAddr(ptr, &mark, 8, "read monitor markword");
                age = (mark >> 3) & 0x1111;
            }
            else
                age = (m_value >> 3) & 0x1111;
            if (age < hwgc->pars.ageThreshold)
                dest_attr_ptr = region_attr_ptr;
        }

        uintptr_t from_region_ptr = hwgc->pars.heapRegionBiasedBase + (obj >> hwgc->pars.heapRegionShiftBy) * 8;
        readHWAddr(from_region_ptr, &from_region, 8, "read from region");
        uint node_index = 0;

        uintptr_t alloc_buffers_ptr = hwgc->pars.plabAllocatorPtr + 0x10;
        uint16_t dest_attr;
        readHWAddr(dest_attr_ptr, &dest_attr, 2, "read dest attr");
        dest_attr_type = (int8_t)(dest_attr >> 8);

        uintptr_t buffer_ptr;
        readHWAddr(alloc_buffers_ptr + dest_attr_type * 8, &buffer_ptr, 8, "read buffer ptr");
        uintptr_t buffer;
        readHWAddr(buffer_ptr, &buffer, 8, "read buffer");

        uintptr_t region_top, region_end;
        readHWAddr(buffer + 0x30, &region_top, 8, "read region_top");
        readHWAddr(buffer + 0x38, &region_end, 8, "read region_end");
        if ((region_end - region_top) / 8 >= size)
        {
            obj_ptr = region_top;
            uintptr_t write_top_res = region_top + size * 8;
            writeHWAddr(buffer + 0x30, &write_top_res, 8, "write region_top");
            hwgc->current = STEP_Copy2SurvivorAop;
        }
        else
        {
            obj_ptr = 0;
            hwgc->current = STEP_ALLOC_INT;
            hwgc->softPars.par0 = dest_attr_ptr;
            hwgc->softPars.par1 = obj;
            hwgc->softPars.par2 = size;
            hwgc->softPars.par3 = (uint64_t)age << 32 | node_index;
            if (qatomic_read(&hwgc->status) & HWGC_STATUS_IRQ)
            {
                bql_lock();
                hwgc_raise_irq(hwgc, ALLOC_SLOW_IRQ);
                bql_unlock();
            }
        }
    }

    if (hwgc->current == STEP_ALLOC_WAKE)
    {
        obj_ptr = hwgc->soft_res;
        hwgc->current = STEP_Copy2SurvivorAop;
    }

    static uintptr_t start_map = 0, end_map = 0;
    if (hwgc->current == STEP_Copy2SurvivorAop)
    {
        uintptr_t m = (obj_ptr & ~0x3) | 0x3;
        writeHWAddr(obj, &m, 8, "write obj markword");

        uint youngIndex;
        readHWAddr(from_region + 256, &youngIndex, 4, "read youngindex");

        uint64_t orginValue;
        readHWAddr(hwgc->pars.youngWordsBase + youngIndex * 8, &orginValue, 8, "read orgin");
        uint64_t writeValue = orginValue + size;
        writeHWAddr(hwgc->pars.youngWordsBase + youngIndex * 8, &writeValue, 8, "write youngWordsBase");

        uint64_t new_mark = m_value;
        if (dest_attr_type == 0)
        {
            if ((m_value & 0x1) == 0x0)
            {
                bool has_monitor = m_value & 0x2;
                uint64_t ptr = has_monitor ? m_value ^ 0x2 : m_value;
                uint64_t mark;
                readHWAddr(ptr, &mark, 8, "read monitor markword");
                mark = (mark & ~0x3) |
                       (((age + 1 < 15 ? age + 1 : age) & 15) << 3);
                writeHWAddr(ptr, &mark, 8, "write monitor markword");
            }
            else
                new_mark = (m_value & ~0x3) |
                           (((age + 1 < 15 ? age + 1 : age) & 15) << 3);
        }
        writeHWAddr(obj_ptr, &new_mark, 8, "write dest markword");

        for (size_t i = 1; i < size; ++i)
        {
            uintptr_t res;
            readHWAddr(obj + i * 8, &res, 8, "read src entry");
            writeHWAddr(obj_ptr + i * 8, &res, 8, "write dest entry");
        }

        scanning_in_young = dest_attr_type == 0;

        from_obj = obj;
        to_obj = obj_ptr;

        if (lh < 0)
        {
            if (kid == 5)
            {
                int array_length;
                readHWAddr(from_obj + 16, &array_length, 4, "read array length");
                uint64_t end = array_length % hwgc->pars.chunkSize;
                writeHWAddr(to_obj + 16, &end, 4, "write dest array length");

                uint step_index = end;
                uint step_ncreate = array_length > end ? 1u : 0u;
                for (uint i = 0; i < step_ncreate; ++i)
                    pushTask(hwgc, from_obj + 0x2);

                uintptr_t low = to_obj + 24;
                uintptr_t high = low + step_index * 8;
                p = to_obj + 24;
                q = p + array_length * 8;
                if (p < low)
                    p = low;
                if (q > high)
                    q = high;
                obj = obj_ptr;
                hwgc->current = STEP_TRACE_PLUS;
                hwgc->done_to = STEP_UPDATE_REF;
            }
            else
            {
                obj = obj_ptr;
                hwgc->current = STEP_UPDATE_REF;
            }
        }
        else
        {
            int vtable_len, itable_len, nonStaticOopMapSize;
            readHWAddr(klass_ptr + 160, &vtable_len, 4, "read vtable len");
            uint64_t temp;
            readHWAddr(klass_ptr + 296, &temp, 8, "read itable_len nonStaticOopMapSize");
            itable_len = temp >> 32;
            nonStaticOopMapSize = (int)temp;
            start_map = klass_ptr + 464 + (vtable_len + itable_len) * 8;
            end_map = start_map + nonStaticOopMapSize * 8;
            hwgc->current = STEP_OOP_TRACE;
        }
    }

    static int ref_state = 0;
    if (hwgc->current == STEP_OOP_TRACE)
    {
        if (start_map < end_map)
        {
            end_map -= 8;
            uint64_t temp;
            readHWAddr(end_map, &temp, 8, "read end map");
            p = obj_ptr + (int)temp;
            q = p + (temp >> 32) * 8;
            hwgc->current = STEP_TRACE_DEC;
            hwgc->done_to = STEP_OOP_TRACE;
        }
        else
        {
            ref_state = 0;
            if (kid == 2)
                hwgc->current = STEP_MIRROR_TRACE;
            else if (kid == 1)
                hwgc->current = STEP_REF_TRACE;
            else
            {
                obj = obj_ptr;
                hwgc->current = STEP_UPDATE_REF;
            }
        }
    }

    if (hwgc->current == STEP_TRACE_DEC)
    {
        if (p < q)
        {
            q -= 8;
            src = q - to_obj + from_obj;
            dest = q;
            hwgc->current = STEP_DO_OOP_WORK;
            hwgc->previous = STEP_TRACE_DEC;
        }
        else
            hwgc->current = hwgc->done_to;
    }

    if (hwgc->current == STEP_MIRROR_TRACE)
    {
        uint staticCount;
        readHWAddr(from_obj + 40, &staticCount, 4, "read staticCount");
        p = to_obj + 184;
        q = p + staticCount * 8;
        obj = obj_ptr;
        hwgc->current = STEP_TRACE_PLUS;
        hwgc->done_to = STEP_UPDATE_REF;
    }

    if (hwgc->current == STEP_REF_TRACE)
    {
        if (ref_state == 0)
        {
            src = from_obj + 40;
            dest = to_obj + 40;
            hwgc->current = STEP_DO_OOP_WORK;
            hwgc->previous = STEP_REF_TRACE;
            ref_state = 1;
        }
        else if (ref_state == 1)
        {
            src = from_obj + 16;
            dest = to_obj + 16;
            hwgc->current = STEP_DO_OOP_WORK;
            hwgc->previous = STEP_REF_TRACE;
            ref_state = 2;
        }
        else if (ref_state == 2)
        {
            src = from_obj + 40;
            dest = to_obj + 40;
            hwgc->current = STEP_DO_OOP_WORK;
            hwgc->previous = STEP_UPDATE_REF;

            obj = obj_ptr;
            ref_state = 0;
        }
    }
}

static void *hwgc_work_thread(void *opaque)
{
    HWGCState *hwgc = opaque;

    while (1)
    {
        printf("do hwgc work\n");
        qemu_mutex_lock(&hwgc->thr_mutex);
        while ((qatomic_read(&hwgc->status) & HWGC_STATUS_COMPUTING) == 0 && !hwgc->stop)
            qemu_cond_wait(&hwgc->thr_cond, &hwgc->thr_mutex);

        if (hwgc->stop)
        {
            qemu_mutex_unlock(&hwgc->thr_mutex);
            break;
        }

        qemu_mutex_unlock(&hwgc->thr_mutex);

        while (1)
        {
            do_hwgc_work(hwgc);
            if (hwgc->current == STEP_DONE)
                break;
            if (hwgc->current == STEP_ALLOC_INT)
            {
                qemu_mutex_lock(&hwgc->thr_mutex);
                while (!hwgc->cont && !hwgc->stop)
                    qemu_cond_wait(&hwgc->thr_cond, &hwgc->thr_mutex);
                if (hwgc->stop)
                {
                    qemu_mutex_unlock(&hwgc->thr_mutex);
                    return NULL;
                }
                printf("wait soft alloc finished\n");
                hwgc->cont = false;
                hwgc->current = STEP_ALLOC_WAKE;
                qemu_mutex_unlock(&hwgc->thr_mutex);
                continue;
            }
            if (hwgc->current == STEP_ENQUEUED_INT)
            {
                qemu_mutex_lock(&hwgc->thr_mutex);
                while (!hwgc->cont && !hwgc->stop)
                    qemu_cond_wait(&hwgc->thr_cond, &hwgc->thr_mutex);
                if (hwgc->stop)
                {
                    qemu_mutex_unlock(&hwgc->thr_mutex);
                    return NULL;
                }
                printf("wait soft enqueued finished\n");
                hwgc->cont = false;
                hwgc->current = STEP_UPDATE_CARD;
                qemu_mutex_unlock(&hwgc->thr_mutex);
                continue;
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

    // 设置 设备停止
    qemu_mutex_lock(&hwgc->thr_mutex);
    hwgc->stop = true;
    qemu_mutex_unlock(&hwgc->thr_mutex);

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