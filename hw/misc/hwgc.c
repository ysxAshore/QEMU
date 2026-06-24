#include "hwgc.h"

static uint32_t hwgc_tlb_hash(uint64_t va_page) { return (va_page >> HWGC_PAGE_SHIFT) & (HWGC_TLB_SIZE - 1); }

static void hwgc_tlb_flush(HWGCDevState *s) { memset(s->tlb, 0, sizeof(s->tlb)); }

static void hwgc_tlb_insert_locked(HWGCDevState *s, uint64_t va, hwaddr pa)
{
    uint64_t va_page;
    hwaddr pa_page;
    uint32_t idx;

    va_page = va & ~HWGC_PAGE_MASK;
    pa_page = pa & ~(hwaddr)HWGC_PAGE_MASK;
    idx = hwgc_tlb_hash(va_page);

    s->tlb[idx].valid = true;
    s->tlb[idx].va_page = va_page;
    s->tlb[idx].pa_page = pa_page;

    IFDEF(TRACE, printf("[hwgc] TLB insert: idx=%u va_page=0x%016" PRIx64
                        " pa_page=0x%016" HWADDR_PRIx "\n",
                        idx, va_page, pa_page));
}

static bool hwgc_tlb_lookup_locked(HWGCDevState *s, uint64_t va, hwaddr *pa)
{
    uint64_t va_page;
    uint64_t offset;
    uint32_t idx;
    HWGCTLBEntry *e;

    va_page = va & ~HWGC_PAGE_MASK;
    offset = va & HWGC_PAGE_MASK;
    idx = hwgc_tlb_hash(va_page);
    e = &s->tlb[idx];

    if (!e->valid || e->va_page != va_page)
        return false;

    *pa = e->pa_page | offset;
    return true;
}

// PCI/PCIe 支持两种中断机制
//      MSI: 边沿出发 不需要维持状态 发一次会自动无效掉
//      Legacy INTx: 电平触发 需要手动拉低 hwgc_lower_irq
// 检查是否启用 msi function
static bool hwgc_msi_enabled(HWGCDevState *s) { return msi_enabled(&s->pdev); }

static void hwgc_raise_irq_from_worker(HWGCDevState *s, uint32_t bits)
{
    qatomic_or(&s->irq_status, bits);

    if (!(qatomic_read(&s->status) & ST_IRQ_EN))
        return;

    bql_lock();
    if (hwgc_msi_enabled(s))
        msi_notify(&s->pdev, 0);
    else
        pci_set_irq(&s->pdev, 1);
    bql_unlock();
}

static void hwgc_lower_irq_from_mmio(HWGCDevState *s, uint32_t bits)
{
    qatomic_and(&s->irq_status, ~bits);

    if (!qatomic_read(&s->irq_status) && !hwgc_msi_enabled(s))
        pci_set_irq(&s->pdev, 0);
}

static void hwgc_arm_timer(HWGCDevState *s)
{
    timer_mod_ns(s->tick_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + s->period_ns);
}

static void hwgc_pause_for_tlb_miss(HWGCDevState *s, uint64_t va, uint32_t access)
{
    s->irq_par0 = va;
    s->irq_par1 = access;
    s->timer_running = false;
    qatomic_or(&s->status, ST_WAIT_TLB);

    hwgc_raise_irq_from_worker(s, IRQ_TLB_MISS);
}

/*
 * Device-side translation: VA -> TLB lookup -> PA.
 * If miss, raise IRQ and pause. The driver will fill REG_TLB_FILL_* and
 * write CMD_CONTINUE, after which the same stage is retried.
 */
static bool hwgc_translate(HWGCDevState *s, uint64_t va, uint32_t access, hwaddr *pa)
{
    bool hit;

    hit = hwgc_tlb_lookup_locked(s, va, pa);

    if (!hit)
    {
        printf("translate not hit\n");
        hwgc_pause_for_tlb_miss(s, va, access);
        return false;
    }

    return true;
}

static bool hwgc_access(HWGCDevState *s, uint64_t va, void *val, uint size, bool write)
{
    hwaddr pa;
    MemTxResult tx;

    int align = size - 1;

    // demo check va必须8B对齐 且访问必须在一页内
    if ((va & align) != 0 || ((va & (HWGC_PAGE_SIZE - 1)) > HWGC_PAGE_SIZE - size))
    {
        printf("va address error\n");
        qatomic_or(&s->status, ST_ERROR);
        s->timer_running = false;
        hwgc_raise_irq_from_worker(s, IRQ_ERROR);
        return false;
    }

    if (!hwgc_translate(s, va, write ? ACCESS_WRITE : ACCESS_READ, &pa))
        return false;

    bql_lock();
    if (write)
        tx = address_space_write(&address_space_memory, pa, MEMTXATTRS_UNSPECIFIED, (uint8_t *)val, size);
    else
        tx = address_space_read(&address_space_memory, pa, MEMTXATTRS_UNSPECIFIED, (uint8_t *)val, size);
    bql_unlock();

    if (tx != MEMTX_OK)
    {
        printf("memory access error\n");
        qatomic_or(&s->status, ST_ERROR);
        s->timer_running = false;
        hwgc_raise_irq_from_worker(s, IRQ_ERROR);
        return false;
    }
    return true;
}

static bool hwgc_cmpxchg(HWGCDevState *s, uintptr_t vaddr, uint64_t old_val, uint64_t new_val, int size, void *return_value)
{
    // demo check va必须8B对齐 且访问必须在一页内
    if ((vaddr & 7) != 0 || ((vaddr & (HWGC_PAGE_SIZE - 1)) > HWGC_PAGE_SIZE - sizeof(uint64_t)))
    {
        printf("va address error\n");
        qatomic_or(&s->status, ST_ERROR);
        s->timer_running = false;
        hwgc_raise_irq_from_worker(s, IRQ_ERROR);
        return false;
    }

    hwaddr paddr;
    if (!hwgc_translate(s, vaddr, true, &paddr))
        return false;

    RCU_READ_LOCK_GUARD();

    hwaddr xlat = paddr;
    hwaddr len = size;
    MemoryRegion *mr = address_space_translate(&address_space_memory, paddr, &xlat, &len, true, MEMTXATTRS_UNSPECIFIED);
    if (!memory_access_is_direct(mr, true, MEMTXATTRS_UNSPECIFIED) || len < size)
    {
        printf("memory region not is ram\n");

        qatomic_or(&s->status, ST_ERROR);
        s->timer_running = false;

        hwgc_raise_irq_from_worker(s, IRQ_ERROR);

        return false;
    }

    uint8_t *host_base = memory_region_get_ram_ptr(mr);
    if (size == 8)
    {
        uint64_t *host_p = (uint64_t *)(host_base + xlat);
        *(uint64_t *)return_value = qatomic_cmpxchg(host_p, old_val, new_val);
        if (*(uint64_t *)return_value == old_val)
            memory_region_set_dirty(mr, xlat, sizeof(uint64_t));
        return true;
    }
    else if (size == 4)
    {
        uint *host_p = (uint *)(host_base + xlat);
        *(uint *)return_value = qatomic_cmpxchg(host_p, (uint)old_val, (uint)new_val);
        if (*(uint *)return_value == (uint)old_val)
            memory_region_set_dirty(mr, xlat, sizeof(uint));
        return true;
    }
    else
    {
        printf("not supported size\n");

        qatomic_or(&s->status, ST_ERROR);
        s->timer_running = false;

        hwgc_raise_irq_from_worker(s, IRQ_ERROR);
        return false;
    }
}

static inline void hwgc_goto_stage(HWGCDevState *s, int stage, int sub_stage)
{
    s->stage = stage;
    s->sub_stage = sub_stage;
}

static inline void hwgc_return_previous(HWGCDevState *s)
{
    s->stage = s->stageData.previous;
    s->sub_stage = s->stageData.previous_sub_stage;
}

static inline void hwgc_return_done(HWGCDevState *s)
{
    s->stage = s->stageData.done_to;
    s->sub_stage = s->stageData.doneto_sub_stage;
}

static void stage_fetch_function(HWGCDevState *s)
{
    assert(s->sub_stage == 0);
    struct HWGCStageData *d = &s->stageData;

    if (d->localBot == 0)
    {
        hwgc_goto_stage(s, STAGE_DONE, 0);
        return;
    }

    uint elems_bias = (d->localBot - 1) & ((1 << 17) - 1);
    printf("elems bias %x, addr %lx\n", elems_bias, d->pars.taskQueueElemsBase + elems_bias * 8);
    if (!hwgc_access(s, d->pars.taskQueueElemsBase + elems_bias * 8, &d->task, 8, false))
        return;
    d->localBot = elems_bias;

    if ((d->task & 0x3) == 0x2)
    {
        d->task -= 0x2;
        hwgc_goto_stage(s, STAGE_PARTIAL_ARRAY, 0);
    }
    else
    {
        d->task -= (d->task & 0x3);
        hwgc_goto_stage(s, STAGE_COMMON_OOP, 0);
    }
    printf("dispatch task %lx\n", d->task);
}

static void stage_partial_array_function(HWGCDevState *s)
{
    struct HWGCStageData *d = &s->stageData;

    IFDEF(TRACE, printf("[PARTIAL_ARRAY] sub=%u task=%lx from=%lx to=%lx localBot=%u\n",
                        s->sub_stage, d->task, d->from_obj, d->to_obj, d->localBot));

    switch (s->sub_stage)
    {
    case 0:
    {
        d->from_obj = d->task;

        IFDEF(TRACE, printf("[PARTIAL_ARRAY:0] read forwarding mark, from=%lx\n",
                            d->from_obj));

        if (!hwgc_access(s, d->from_obj, &d->partial_m_value, 8, false))
            return;

        d->to_obj = d->partial_m_value & ~0x3;
        uintptr_t from_len_addr = d->from_obj +
                                  (d->pars.useCompressedKlassPointers ? 12 : 16);

        if (!hwgc_access(s, from_len_addr, &d->partial_from_length, 4, false))
            return;

        IFDEF(TRACE, printf("[PARTIAL_ARRAY:0] mark=%lx to=%lx src_len=%u len_addr=%lx\n",
                            d->partial_m_value, d->to_obj,
                            d->partial_from_length, from_len_addr));

        s->sub_stage = 1;
        break;
    }

    case 1:
    {
        int temp;
        uintptr_t to_len_addr = d->to_obj +
                                (d->pars.useCompressedKlassPointers ? 12 : 16);

        if (!hwgc_access(s, to_len_addr, &d->start, 4, false))
            return;

        temp = d->start + d->pars.chunkSize;

        IFDEF(TRACE, printf("[PARTIAL_ARRAY:1] dst_len=%u chunk=%u -> %d, addr=%lx\n",
                            d->start, (unsigned)d->pars.chunkSize,
                            temp, to_len_addr));

        if (!hwgc_access(s, to_len_addr, &temp, 4, true))
            return;

        s->sub_stage = 2;
        break;
    }

    case 2:
    {
        uint32_t task_num = d->start / d->pars.chunkSize;
        uint32_t remaining_tasks =
            (d->partial_from_length - d->start) / d->pars.chunkSize;
        uint32_t task_limit = (uint32_t)d->pars.stepperOffset;
        uint32_t task_fanout = d->pars.stepperOffset >> 32;
        uint32_t max_pending = (task_fanout - 1) * task_num + 1;
        uint32_t pending =
            MIN(max_pending, MIN(remaining_tasks, task_limit));

        d->ncreate = MIN(task_fanout,
                         MIN(remaining_tasks, task_limit + 1) - pending);
        d->i = 0;

        IFDEF(TRACE, printf("[PARTIAL_ARRAY:2] start=%u src_len=%u task_num=%u "
                            "remaining=%u limit=%u fanout=%u pending=%u create=%u\n",
                            d->start, d->partial_from_length, task_num,
                            remaining_tasks, task_limit, task_fanout,
                            pending, d->ncreate));

        IFDEF(TRACE, if (d->start > d->partial_from_length)
                         printf("[PARTIAL_ARRAY:2] WARNING: start > source length\n"));

        s->sub_stage = 3;
        break;
    }

    case 3:
    {
        if (d->i >= d->ncreate)
        {
            IFDEF(TRACE, printf("[PARTIAL_ARRAY:3] fanout finished, created=%u\n",
                                d->i));
            s->sub_stage = 4;
            break;
        }

        uintptr_t pushData = d->from_obj + 0x2;
        uintptr_t queue_addr =
            d->pars.taskQueueElemsBase + d->localBot * 8;

        IFDEF(TRACE, printf("[PARTIAL_ARRAY:3] push task=%lx queue[%u]=%lx\n",
                            pushData, d->localBot, queue_addr));

        if (!hwgc_access(s, queue_addr, &pushData, 8, true))
            return;

        d->localBot = (d->localBot + 1) & ((1 << 17) - 1);
        d->i++;

        IFDEF(TRACE, printf("[PARTIAL_ARRAY:3] pushed, next localBot=%u i=%u\n",
                            d->localBot, d->i));
        break;
    }

    case 4:
    {
        uintptr_t heap_region_ptr = d->pars.heapRegionBiasedBase +
                                    (d->to_obj >> d->pars.heapRegionShiftBy) * 8;

        IFDEF(TRACE, printf("[PARTIAL_ARRAY:4] lookup region, to=%lx region_ptr=%lx\n",
                            d->to_obj, heap_region_ptr));

        if (!hwgc_access(s, heap_region_ptr, &d->heap_region, 8, false))
            return;

        IFDEF(TRACE, printf("[PARTIAL_ARRAY:4] heap_region=%lx\n",
                            d->heap_region));

        s->sub_stage = 5;
        break;
    }

    case 5:
        if (!hwgc_access(s, d->heap_region + 0xbc,
                         &d->heap_region_type, 4, false))
            return;

        IFDEF(TRACE, printf("[PARTIAL_ARRAY:5] region_type=0x%x\n",
                            d->heap_region_type));

        s->sub_stage = 6;
        break;

    case 6:
    {
        d->scanning_in_young = (d->heap_region_type & 0x2) != 0;

        size_t oop_size = d->pars.useCompressedOops ? 4 : 8;
        uintptr_t base = d->to_obj +
                         (d->pars.useCompressedKlassPointers ? 16 : 24);

        uintptr_t low = base + d->start * oop_size;
        uintptr_t high = base +
                         (d->start + d->pars.chunkSize) * oop_size;

        d->p = base;
        d->q = base +
               (d->start + d->pars.chunkSize) * oop_size;

        if (d->p < low)
            d->p = low;

        if (d->q > high)
            d->q = high;

        IFDEF(TRACE, printf("[PARTIAL_ARRAY:6] young=%d oop_size=%zu "
                            "base=%lx start=%u chunk=%u p=%lx q=%lx\n",
                            d->scanning_in_young, oop_size, base,
                            d->start, (unsigned)d->pars.chunkSize,
                            d->p, d->q));

        d->previous = STAGE_TRACE_PLUS;
        d->previous_sub_stage = 0;

        d->done_to = STAGE_FETCH;
        d->doneto_sub_stage = 0;

        IFDEF(TRACE, printf("[PARTIAL_ARRAY:6] goto TRACE_PLUS -> FETCH\n"));

        hwgc_goto_stage(s, STAGE_TRACE_PLUS, 0);
        break;
    }

    default:
        IFDEF(TRACE, printf("[PARTIAL_ARRAY] invalid sub_stage=%u, reset\n",
                            s->sub_stage));
        hwgc_goto_stage(s, STAGE_PARTIAL_ARRAY, 0);
        break;
    }
}

static void stage_oop_function(HWGCDevState *s)
{
    struct HWGCStageData *d = &s->stageData;

    IFDEF(TRACE, printf("[COMMON_OOP] sub=%u task=%lx offset=%lx from=%lx to=%lx\n",
                        s->sub_stage, d->task, d->offset,
                        d->from_obj, d->to_obj));

    switch (s->sub_stage)
    {
    case 0:
        IFDEF(TRACE, printf("[COMMON_OOP:0] read oop slot, task_addr=%lx\n",
                            d->task));

        uint size = d->pars.useCompressedOops ? 4 : 8;
        if (!hwgc_access(s, d->task, &d->offset, size, false))
            return;

        IFDEF(TRACE, printf("[COMMON_OOP:0] read offset/raw_oop=%lx\n",
                            d->offset));

        s->sub_stage = 1;
        break;

    case 1:
        if (d->pars.useCompressedOops)
        {
            uint32_t narrow_oop = (uint32_t)d->offset;

            if (narrow_oop == 0)
                d->from_obj = 0;
            else
                d->from_obj = (uintptr_t)d->pars.compressedOopBase +
                              ((uintptr_t)narrow_oop << d->pars.compressedOopShift);

            IFDEF(TRACE, printf("[COMMON_OOP:1] compressed oop=%x -> from=%lx "
                                "base=%lx shift=%u\n",
                                narrow_oop, d->from_obj,
                                d->pars.compressedOopBase,
                                d->pars.compressedOopShift));
        }
        else
        {
            d->from_obj = d->offset;

            IFDEF(TRACE, printf("[COMMON_OOP:1] uncompressed from=%lx\n",
                                d->from_obj));
        }

        if (!hwgc_access(s, d->from_obj, &d->common_m_value, 8, false))
            return;

        IFDEF(TRACE, printf("[COMMON_OOP:1] mark/forwarding=%lx\n",
                            d->common_m_value));

        s->sub_stage = 2;
        break;

    case 2:
        d->region_attr_ptr = d->pars.regionAttrBiasedBase +
                             (d->from_obj >> d->pars.regionAttrShiftBy) * 2;

        IFDEF(TRACE, printf("[COMMON_OOP:2] from=%lx region_attr_ptr=%lx\n",
                            d->from_obj, d->region_attr_ptr));

        if (!hwgc_access(s, d->region_attr_ptr, &d->src_region_attr, 2, false))
            return;

        IFDEF(TRACE, printf("[COMMON_OOP:2] src_region_attr=0x%x type=%d\n",
                            d->src_region_attr,
                            (int8_t)(d->src_region_attr >> 8)));

        if ((int8_t)(d->src_region_attr >> 8) < 0)
        {
            IFDEF(TRACE, printf("[COMMON_OOP:2] source already processed, "
                                "goto FETCH\n"));
            hwgc_goto_stage(s, STAGE_FETCH, 0);
            return;
        }

        s->sub_stage = 3;
        break;

    case 3:
        if ((d->common_m_value & 0x3) == 0x3)
        {
            d->to_obj = d->common_m_value & ~0x3;

            IFDEF(TRACE, printf("[COMMON_OOP:3] forwarded: mark=%lx -> to=%lx\n",
                                d->common_m_value, d->to_obj));

            s->sub_stage = 4;
        }
        else
        {
            d->src_region_attr_ptr = d->region_attr_ptr;

            IFDEF(TRACE, printf("[COMMON_OOP:3] not forwarded, "
                                "goto COPY2SURVIVOR from=%lx attr_ptr=%lx\n",
                                d->from_obj, d->src_region_attr_ptr));

            hwgc_goto_stage(s, STAGE_COPY2SURVIVOR, 0);
        }
        break;

    case 4:
    {
        uintptr_t writeObj = d->to_obj;

        if (d->pars.useCompressedOops)
        {
            uint32_t narrow_writeObj;

            writeObj = (d->to_obj - d->pars.compressedOopBase) >>
                       d->pars.compressedOopShift;
            narrow_writeObj = (uint32_t)writeObj;

            IFDEF(TRACE, printf("[COMMON_OOP:4] write compressed oop=%x "
                                "to task_addr=%lx, to=%lx\n",
                                narrow_writeObj, d->task, d->to_obj));

            if (!hwgc_access(s, d->task, &narrow_writeObj, 4, true))
                return;
        }
        else
        {
            IFDEF(TRACE, printf("[COMMON_OOP:4] write oop=%lx to task_addr=%lx\n",
                                writeObj, d->task));

            if (!hwgc_access(s, d->task, &writeObj, 8, true))
                return;
        }

        s->sub_stage = 5;
        break;
    }

    case 5:
    {
        uintptr_t heap_region_ptr;
        uintptr_t task_region = d->task >> d->pars.logOfHRGrainBytes;
        uintptr_t to_region = d->to_obj >> d->pars.logOfHRGrainBytes;

        IFDEF(TRACE, printf("[COMMON_OOP:5] task_region=%lx to_region=%lx\n",
                            task_region, to_region));

        if (((d->task ^ d->to_obj) >> d->pars.logOfHRGrainBytes) == 0)
        {
            IFDEF(TRACE, printf("[COMMON_OOP:5] same region, goto FETCH\n"));
            hwgc_goto_stage(s, STAGE_FETCH, 0);
            return;
        }

        heap_region_ptr = d->pars.heapRegionBiasedBase +
                          (d->task >> d->pars.heapRegionShiftBy) * 8;

        IFDEF(TRACE, printf("[COMMON_OOP:5] heap_region_ptr=%lx\n",
                            heap_region_ptr));

        if (!hwgc_access(s, heap_region_ptr, &d->heap_region, 8, false))
            return;

        IFDEF(TRACE, printf("[COMMON_OOP:5] heap_region=%lx\n",
                            d->heap_region));

        s->sub_stage = 6;
        break;
    }

    case 6:
    {
        uintptr_t attr_ptr;

        if (!hwgc_access(s, d->heap_region + 0xbc,
                         &d->heap_region_type, 4, false))
            return;

        IFDEF(TRACE, printf("[COMMON_OOP:6] heap_region_type=0x%x\n",
                            d->heap_region_type));

        if ((d->heap_region_type & 0x2) != 0)
        {
            IFDEF(TRACE, printf("[COMMON_OOP:6] destination is young, "
                                "goto FETCH\n"));
            hwgc_goto_stage(s, STAGE_FETCH, 0);
            return;
        }

        attr_ptr = d->pars.regionAttrBiasedBase +
                   (d->to_obj >> d->pars.regionAttrShiftBy) * 2;

        if (!hwgc_access(s, attr_ptr, &d->aop_region_attr, 2, false))
            return;

        d->aop_dest = d->task;

        IFDEF(TRACE, printf("[COMMON_OOP:6] AOP: dest_slot=%lx to=%lx "
                            "attr_ptr=%lx attr=0x%x\n",
                            d->aop_dest, d->to_obj,
                            attr_ptr, d->aop_region_attr));

        d->previous = STAGE_FETCH;
        d->previous_sub_stage = 0;

        IFDEF(TRACE, printf("[COMMON_OOP:6] goto AOP_WORK\n"));

        hwgc_goto_stage(s, STAGE_AOP_WORK, 0);
        break;
    }

    default:
        IFDEF(TRACE, printf("[COMMON_OOP] invalid sub=%u, reset COMMON_OOP\n",
                            s->sub_stage));
        hwgc_goto_stage(s, STAGE_COMMON_OOP, 0);
        break;
    }
}

static void stage_copy2survivor_function(HWGCDevState *s)
{
    struct HWGCStageData *d = &s->stageData;
    int8_t src_region_attr_type = (int8_t)(d->src_region_attr >> 8);

    IFDEF(TRACE, printf("[COPY2SURVIVOR] sub=%u from=%lx to=%lx mark=%lx "
                        "src_attr=0x%x src_type=%d\n",
                        s->sub_stage, d->from_obj, d->to_obj,
                        d->common_m_value, d->src_region_attr,
                        src_region_attr_type));

    switch (s->sub_stage)
    {
    case 0:
    {
        IFDEF(TRACE, printf("[COPY2SURVIVOR:0] read klass from=%lx\n",
                            d->from_obj));

        if (!hwgc_access(s, d->from_obj + 8, &d->klass_ptr, 8, false))
            return;

        IFDEF(TRACE, printf("[COPY2SURVIVOR:0] raw klass=%lx\n",
                            d->klass_ptr));

        s->sub_stage = 1;
        break;
    }

    case 1:
    {
        if (d->pars.useCompressedKlassPointers)
        {
            d->klass_ptr = d->pars.compressedKlassPointerBase +
                           ((uintptr_t)((uint32_t)d->klass_ptr) << d->pars.compressedKlassPointerShift);
        }

        IFDEF(TRACE, printf("[COPY2SURVIVOR:1] klass=%lx, read layout helper\n",
                            d->klass_ptr));

        if (!hwgc_access(s, d->klass_ptr + 8, &d->region_attr_ptr, 8, false))
            return;

        IFDEF(TRACE, printf("[COPY2SURVIVOR:1] layout helper=%lx\n",
                            d->region_attr_ptr));

        s->sub_stage = 2;
        break;
    }

    case 2:
    {
        d->lh = (int)d->region_attr_ptr;
        d->kid = (int)(d->region_attr_ptr >> 32);

        if (d->lh > 0 && (d->lh & 0x1) == 0)
        {
            d->size = (size_t)d->lh >> 3;
        }
        else if (d->lh < 0)
        {
            uintptr_t len_addr = d->from_obj +
                                 (d->pars.useCompressedKlassPointers ? 12 : 16);

            if (!hwgc_access(s, len_addr,
                             &d->common_oop_array_length, 4, false))
                return;

            size_t temp = ((size_t)d->common_oop_array_length << (uint8_t)d->lh) +
                          (uint8_t)(d->lh >> 16);

            d->size = (temp & 0x7) ? ((temp >> 3) + 1) : (temp >> 3);

            IFDEF(TRACE, printf("[COPY2SURVIVOR:2] array len=%u len_addr=%lx\n",
                                d->common_oop_array_length, len_addr));
        }
        else
        {
            if (d->kid == 2)
            {
                uint oop_size_offset = d->pars.useCompressedKlassPointers ? 0x20 : 0x24;
                if (!hwgc_access(s, d->from_obj + oop_size_offset, &d->size, 8, false))
                    return;
            }
            else
            {
                d->size = (size_t)d->lh >> 3;
            }
        }

        IFDEF(TRACE, printf("[COPY2SURVIVOR:2] lh=%d kid=%d size=%zu words\n",
                            d->lh, d->kid, d->size));

        s->sub_stage = 3;
        break;
    }

    case 3:
    {
        if (!hwgc_access(s, d->pars.pss + 0x178,
                         &d->dest_attr_cache, 4, false))
            return;

        d->dest_attr = src_region_attr_type == 1 ? d->dest_attr_cache >> 16 : d->dest_attr_cache & 0xffff;

        d->dest_attr_ptr = src_region_attr_type == 1 ? d->pars.pss + 0x178 + 0x2 : d->pars.pss + 0x178;

        IFDEF(TRACE, printf("[COPY2SURVIVOR:3] attr_cache=0x%x dest_attr=0x%x "
                            "dest_type=%d dest_attr_ptr=%lx\n",
                            d->dest_attr_cache, d->dest_attr,
                            (int8_t)(d->dest_attr >> 8),
                            d->dest_attr_ptr));

        s->sub_stage = 4;
        break;
    }

    case 4:
    {
        if (src_region_attr_type == 0)
        {
            if ((d->common_m_value & 0x1) == 0x0)
            {
                uintptr_t ptr = (d->common_m_value & 0x2) ? (d->common_m_value ^ 0x2) : d->common_m_value;

                if (!hwgc_access(s, ptr, &d->monitor_markWord, 8, false))
                    return;

                d->age = (d->monitor_markWord >> 3) & 0x1111;

                IFDEF(TRACE, printf("[COPY2SURVIVOR:4] monitor=%lx mark=%lx "
                                    "age=%u\n",
                                    ptr, d->monitor_markWord, d->age));
            }
            else
            {
                d->age = (d->common_m_value >> 3) & 0x1111;

                IFDEF(TRACE, printf("[COPY2SURVIVOR:4] normal mark=%lx age=%u\n",
                                    d->common_m_value, d->age));
            }

            if (!hwgc_access(s, d->pars.pss + 0x17c, &d->pars.ageThreshold, 4, false))
                return;

            if (d->age < d->pars.ageThreshold)
            {
                d->dest_attr = d->src_region_attr;
                d->dest_attr_ptr = d->src_region_attr_ptr;
            }
            IFDEF(TRACE, printf("[COPY2SURVIVOR:4] age=%u < threshold=%u, "
                                "keep source destination attr=0x%x dest_attr_ptr %lx\n",
                                d->age, d->pars.ageThreshold,
                                d->dest_attr, d->dest_attr_ptr));
        }

        IFDEF(TRACE, printf("[COPY2SURVIVOR:4] final dest_attr=0x%x "
                            "dest_type=%d dest_ptr=%lx\n",
                            d->dest_attr, (int8_t)(d->dest_attr >> 8),
                            d->dest_attr_ptr));

        s->sub_stage = 5;
        break;
    }

    case 5:
    {
        int8_t dest_attr_type = (int8_t)(d->dest_attr >> 8);
        uintptr_t allocator_addr = d->pars.plabAllocatorPtr +
                                   0x10 + dest_attr_type * 8;

        IFDEF(TRACE, printf("[COPY2SURVIVOR:5] dest_type=%d "
                            "allocator_slot=%lx\n",
                            dest_attr_type, allocator_addr));

        if (!hwgc_access(s, allocator_addr, &d->buffer_temp, 8, false))
            return;

        IFDEF(TRACE, printf("[COPY2SURVIVOR:5] buffer_temp=%lx\n",
                            d->buffer_temp));

        s->sub_stage = 6;
        break;
    }

    case 6:
    {
        if (!hwgc_access(s, d->buffer_temp, &d->buffer, 8, false))
            return;

        if (!hwgc_access(s, d->buffer + 0x30, &d->region_top, 8, false))
            return;

        if (!hwgc_access(s, d->buffer + 0x38, &d->region_end, 8, false))
            return;

        IFDEF(TRACE, printf("[COPY2SURVIVOR:6] buffer=%lx top=%lx end=%lx "
                            "available=%zu words need=%zu words\n",
                            d->buffer, d->region_top, d->region_end,
                            (d->region_end - d->region_top) / 8,
                            d->size));

        s->sub_stage = 7;
        break;
    }

    case 7:
    {
        if ((d->region_end - d->region_top) / 8 >= d->size)
        {
            uintptr_t writeValue = d->region_top + d->size * 8;

            d->to_obj = d->region_top;

            IFDEF(TRACE, printf("[COPY2SURVIVOR:7] PLAB allocate "
                                "to=%lx size=%zu old_top=%lx new_top=%lx\n",
                                d->to_obj, d->size,
                                d->region_top, writeValue));

            if (!hwgc_access(s, d->buffer + 0x30, &writeValue, 8, true))
                return;

            d->region_top = writeValue;
            s->sub_stage = 8;
        }
        else
        {
            d->to_obj = 0;

            IFDEF(TRACE, printf("[COPY2SURVIVOR:7] PLAB insufficient, "
                                "goto ALLOC: available=%zu need=%zu\n",
                                (d->region_end - d->region_top) / 8,
                                d->size));

            hwgc_goto_stage(s, STAGE_ALLOC, 0);
        }
        break;
    }

    case 8:
    {
        uintptr_t updatedMW = (d->to_obj & ~0x3) | 0x3;
        uintptr_t return_value;

        IFDEF(TRACE, printf("[COPY2SURVIVOR:8] CAS from=%lx expected=%lx "
                            "updated=%lx\n",
                            d->from_obj, d->common_m_value, updatedMW));

        if (!hwgc_cmpxchg(s, d->from_obj, d->common_m_value,
                          updatedMW, 8, &return_value))
            return;

        IFDEF(TRACE, printf("[COPY2SURVIVOR:8] CAS returned=%lx %s\n",
                            return_value,
                            return_value == d->common_m_value ? "WIN" : "LOSE"));

        if (return_value == d->common_m_value)
        {
            d->forward_ptr = 0;
            s->sub_stage = 9;
        }
        else
        {
            d->forward_ptr = return_value & ~0x3;
            s->sub_stage = 13;
        }
        break;
    }

    case 9:
    {
        uintptr_t new_mark = d->common_m_value;

        if ((int8_t)(d->dest_attr >> 8) == 0 &&
            (d->common_m_value & 0x1) != 0)
        {
            new_mark = (d->common_m_value & ~(0x1111 << 3)) |
                       ((((d->age + 1) < 15 ? d->age + 1 : d->age) &
                         0x1111)
                        << 3);
        }

        IFDEF(TRACE, printf("[COPY2SURVIVOR:9] init new object "
                            "to=%lx mark=%lx\n",
                            d->to_obj, new_mark));

        if (!hwgc_access(s, d->to_obj, &new_mark, 8, true))
            return;

        s->sub_stage = 10;
        break;
    }

    case 10:
    {
        if ((int8_t)(d->dest_attr >> 8) == 0 &&
            (d->common_m_value & 0x1) == 0)
        {
            uintptr_t ptr = (d->common_m_value & 0x2) ? (d->common_m_value ^ 0x2) : d->common_m_value;

            if (!hwgc_access(s, ptr, &d->region_attr_ptr, 8, false))
                return;

            d->region_attr_ptr = (d->region_attr_ptr & ~(0x1111 << 3)) |
                                 ((((d->age + 1) < 15 ? d->age + 1 : d->age) &
                                   0x1111)
                                  << 3);

            IFDEF(TRACE, printf("[COPY2SURVIVOR:10] update monitor=%lx "
                                "new_mark=%lx\n",
                                ptr, d->region_attr_ptr));

            if (!hwgc_access(s, ptr, &d->region_attr_ptr, 8, true))
                return;
        }

        s->sub_stage = 11;
        break;
    }

    case 11:
    {
        d->i = 1;

        IFDEF(TRACE, printf("[COPY2SURVIVOR:11] goto COPY from=%lx to=%lx "
                            "size=%zu\n",
                            d->from_obj, d->to_obj, d->size));

        hwgc_goto_stage(s, STAGE_COPY, 0);
        break;
    }

    case 12:
    {
        IFDEF(TRACE, printf("[COPY2SURVIVOR:12] copy complete, kid=%d -> %s\n",
                            d->kid,
                            d->kid == 4 ? "COMMON_OOP" : "TRACE"));

        if (d->kid == 4)
        {
            hwgc_goto_stage(s, STAGE_COMMON_OOP, 4);
            return;
        }

        hwgc_goto_stage(s, STAGE_TRACE, 0);
        break;
    }

    case 13:
    {
        if (!hwgc_access(s, d->buffer + 0x28,
                         &d->region_bottom, 8, false))
            return;

        if (!hwgc_access(s, d->buffer + 0x40,
                         &d->region_hard_end, 8, false))
            return;

        IFDEF(TRACE, printf("[COPY2SURVIVOR:13] lost CAS, forward=%lx "
                            "allocated=%lx buffer=[%lx, %lx)\n",
                            d->forward_ptr, d->to_obj,
                            d->region_bottom, d->region_hard_end));

        if (d->to_obj >= d->region_bottom &&
            d->to_obj < d->region_hard_end)
        {
            IFDEF(TRACE, printf("[COPY2SURVIVOR:13] reclaim allocation, "
                                "restore top=%lx\n",
                                d->to_obj));

            if (!hwgc_access(s, d->buffer + 0x30, &d->to_obj, 8, true))
                return;

            d->to_obj = d->forward_ptr;

            hwgc_goto_stage(s, STAGE_COMMON_OOP, 4);
        }
        else
        {
            IFDEF(TRACE, printf("[COPY2SURVIVOR:13] cannot reclaim PLAB, "
                                "create filler object\n"));

            s->sub_stage = 14;
        }
        break;
    }

    case 14:
    {
        uint words = d->size / 8;
        uintptr_t cur_klass = 0;
        uint header_words =
            d->pars.useCompressedKlassPointers ? 2 : 3;

        if (words >= header_words)
        {
            uint payload_size = words - header_words;
            uint32_t len = payload_size * 2;

            uintptr_t array_len_addr = d->to_obj +
                                       (d->pars.useCompressedKlassPointers ? 12 : 16);

            IFDEF(TRACE, printf("[COPY2SURVIVOR:14] filler int[] "
                                "to=%lx words=%u len=%u len_addr=%lx\n",
                                d->to_obj, words, len, array_len_addr));

            if (!hwgc_access(s, array_len_addr, &len, 4, true))
                return;

            cur_klass = d->pars.intArrayKlassObj;
        }
        else if (words > 0)
        {
            cur_klass = d->pars.objectKlass;
        }

        IFDEF(TRACE, printf("[COPY2SURVIVOR:14] filler to=%lx words=%u "
                            "klass=%lx forward_to=%lx\n",
                            d->to_obj, words, cur_klass, d->forward_ptr));

        uintptr_t mark = 0x1;
        if (!hwgc_access(s, d->to_obj, &mark, 8, true))
            return;

        if (d->pars.useCompressedKlassPointers)
        {
            uint32_t narrow_klass = (uint32_t)((cur_klass - s->stageData.pars.compressedKlassPointerBase) >>
                                               s->stageData.pars.compressedKlassPointerShift);

            if (!hwgc_access(s, d->to_obj + 0x8,
                             &narrow_klass, 4, true))
                return;
        }
        else
        {
            if (!hwgc_access(s, d->to_obj + 0x8, &cur_klass, 8, true))
                return;
        }

        d->to_obj = d->forward_ptr;

        IFDEF(TRACE, printf("[COPY2SURVIVOR:14] filler done, "
                            "goto COMMON_OOP:4 to=%lx\n",
                            d->to_obj));

        hwgc_goto_stage(s, STAGE_COMMON_OOP, 4);
        break;
    }

    default:
        IFDEF(TRACE, printf("[COPY2SURVIVOR] invalid sub=%u, reset\n",
                            s->sub_stage));

        hwgc_goto_stage(s, STAGE_COPY2SURVIVOR, 0);
        break;
    }
}

static void stage_alloc_function(HWGCDevState *s)
{
    struct HWGCStageData *d = &s->stageData;

    IFDEF(TRACE, printf("[ALLOC] sub=%u from=%lx to=%lx size=%zu "
                        "dest_attr=0x%x dest_ptr=%lx\n",
                        s->sub_stage, d->from_obj, d->to_obj,
                        d->size, d->dest_attr, d->dest_attr_ptr));

    switch (s->sub_stage)
    {
    case 0:
    {
        d->previous = STAGE_ALLOC;
        d->previous_sub_stage = 1;

        IFDEF(TRACE, printf("[ALLOC:0] goto ALLOCATE_DIRECT:0 "
                            "return_to=ALLOC:1\n"));

        hwgc_goto_stage(s, STAGE_ALLOCATE_DIRECT, 0);
        break;
    }

    case 1:
    {
        if (d->to_obj == 0)
        {
            IFDEF(TRACE, printf("[ALLOC:1] no target object, "
                                "read old PLAB slot=%lx\n",
                                d->pars.plabAllocatorPtr + 0x18));

            if (!hwgc_access(s, d->pars.plabAllocatorPtr + 0x18,
                             &d->buffer_temp, 8, false))
                return;

            IFDEF(TRACE, printf("[ALLOC:1] old PLAB holder=%lx\n",
                                d->buffer_temp));

            s->sub_stage = 2;
        }
        else
        {
            IFDEF(TRACE, printf("[ALLOC:1] direct allocation succeeded "
                                "to=%lx\n",
                                d->to_obj));

            s->sub_stage = 5;
        }
        break;
    }

    case 2:
    {
        if (!hwgc_access(s, d->buffer_temp, &d->buffer, 8, false))
            return;

        if (!hwgc_access(s, d->buffer + 0x30,
                         &d->region_top, 8, false))
            return;

        if (!hwgc_access(s, d->buffer + 0x38,
                         &d->region_end, 8, false))
            return;

        IFDEF(TRACE, printf("[ALLOC:2] old PLAB buffer=%lx top=%lx "
                            "end=%lx available=%zu words\n",
                            d->buffer, d->region_top, d->region_end,
                            d->region_end >= d->region_top ? (d->region_end - d->region_top) / 8 : 0));

        s->sub_stage = 3;
        break;
    }

    case 3:
    {
        uintptr_t write_top;

        d->dest_attr = (d->dest_attr & 0x00ff) | 0x0100;

        IFDEF(TRACE, printf("[ALLOC:3] set dest_attr=0x%x, "
                            "need=%zu words\n",
                            d->dest_attr, d->size));

        if (d->region_end >= d->region_top &&
            (d->region_end - d->region_top) / 8 >= d->size)
        {
            d->to_obj = d->region_top;
            write_top = d->region_top + d->size * 8;

            IFDEF(TRACE, printf("[ALLOC:3] allocate from old PLAB: "
                                "to=%lx old_top=%lx new_top=%lx\n",
                                d->to_obj, d->region_top, write_top));

            if (!hwgc_access(s, d->buffer + 0x30, &write_top, 8, true))
                return;

            d->region_top = write_top;
            s->sub_stage = 4;
        }
        else
        {
            IFDEF(TRACE, printf("[ALLOC:3] old PLAB insufficient, "
                                "request ALLOCATE_DIRECT\n"));

            d->previous = STAGE_ALLOC;
            d->previous_sub_stage = 4;

            hwgc_goto_stage(s, STAGE_ALLOCATE_DIRECT, 0);
        }
        break;
    }

    case 4:
    {
        uint32_t refill_failed_value;
        uint8_t destination_full = 1;

        if (d->plab_refill_failed)
        {
            refill_failed_value = 0;

            IFDEF(TRACE, printf("[ALLOC:4] clear PLAB refill failure "
                                "flag addr=%lx\n",
                                d->pars.pss + 0x17c));

            if (!hwgc_access(s, d->pars.pss + 0x17c,
                             &refill_failed_value, 4, true))
                return;
        }

        IFDEF(TRACE, printf("[ALLOC:4] mark destination allocation "
                            "flag addr=%lx value=%u\n",
                            d->dest_attr_ptr + 1, destination_full));

        if (!hwgc_access(s, d->dest_attr_ptr + 1,
                         &destination_full, 1, true))
            return;

        s->sub_stage = 5;
        break;
    }

    case 5:
    {
        IFDEF(TRACE, printf("[ALLOC:5] goto COPY2SURVIVOR:8 "
                            "to=%lx size=%zu\n",
                            d->to_obj, d->size));

        hwgc_goto_stage(s, STAGE_COPY2SURVIVOR, 8);
        break;
    }

    default:
        IFDEF(TRACE, printf("[ALLOC] invalid sub=%u, reset\n",
                            s->sub_stage));

        hwgc_goto_stage(s, STAGE_ALLOC, 0);
        break;
    }
}

static void stage_allocate_direct_function(HWGCDevState *s)
{
    struct HWGCStageData *d = &s->stageData;
    int8_t dest_attr_type = (int8_t)(d->dest_attr >> 8);

    IFDEF(TRACE, printf("[ALLOCATE_DIRECT] sub=%u dest_attr=0x%x "
                        "dest_type=%d size=%zu to=%lx buffer=%lx\n",
                        s->sub_stage, d->dest_attr, dest_attr_type,
                        d->size, d->to_obj, d->buffer));

    switch (s->sub_stage)
    {
    case 0:
    {
        uintptr_t plab_stats_ptr = d->pars.g1h + 0x250;

        if (dest_attr_type == 1)
            plab_stats_ptr = d->pars.g1h + 0x2d0;

        IFDEF(TRACE, printf("[ALLOCATE_DIRECT:0] g1h=%lx stats_ptr=%lx "
                            "stats_size_addr=%lx\n",
                            d->pars.g1h, plab_stats_ptr,
                            plab_stats_ptr + 0x30));

        if (!hwgc_access(s, plab_stats_ptr + 0x30,
                         &d->region_attr_ptr, 8, false))
            return;

        IFDEF(TRACE, printf("[ALLOCATE_DIRECT:0] raw_plab_word_size=%lx\n",
                            d->region_attr_ptr));

        d->plab_refill_failed = false;

        s->sub_stage = 1;
        break;
    }

    case 1:
    {
        d->plab_word_size =
            MIN(MAX(d->region_attr_ptr, 0x102), 0x40000);
        d->required_in_plab = d->size + 0x2;

        IFDEF(TRACE, printf("[ALLOCATE_DIRECT:1] plab_word_size=%zu "
                            "required_in_plab=%zu object_size=%zu\n",
                            d->plab_word_size, d->required_in_plab,
                            d->size));

        if (!hwgc_access(s, d->pars.plabAllocatorPtr + 0x8,
                         &d->allocator_ptr, 8, false))
            return;

        IFDEF(TRACE, printf("[ALLOCATE_DIRECT:1] allocator_ptr=%lx\n",
                            d->allocator_ptr));

        s->sub_stage = 2;
        break;
    }

    case 2:
    {
        bool may_throw_away_buffer =
            d->required_in_plab * 100 < d->plab_word_size * 0xa;

        IFDEF(TRACE, printf("[ALLOCATE_DIRECT:2] required=%zu desired=%zu "
                            "may_discard=%d\n",
                            d->required_in_plab, d->plab_word_size,
                            may_throw_away_buffer));

        if (d->required_in_plab <= d->plab_word_size &&
            may_throw_away_buffer)
        {
            uintptr_t buffer_slot = d->pars.plabAllocatorPtr +
                                    0x10 + dest_attr_type * 8;

            IFDEF(TRACE, printf("[ALLOCATE_DIRECT:2] read current PLAB "
                                "slot=%lx\n",
                                buffer_slot));

            if (!hwgc_access(s, buffer_slot, &d->buffer_temp, 8, false))
                return;

            IFDEF(TRACE, printf("[ALLOCATE_DIRECT:2] buffer_temp=%lx\n",
                                d->buffer_temp));

            s->sub_stage = 3;
        }
        else
        {
            IFDEF(TRACE, printf("[ALLOCATE_DIRECT:2] skip old PLAB, "
                                "goto ALLOCATE_DURING_GC preparation\n"));
            s->sub_stage = 7;
        }
        break;
    }

    case 3:
    {
        if (!hwgc_access(s, d->buffer_temp, &d->buffer, 8, false))
            return;

        if (!hwgc_access(s, d->buffer + 0x30, &d->region_top, 8, false))
            return;

        if (!hwgc_access(s, d->buffer + 0x40,
                         &d->region_hard_end, 8, false))
            return;

        IFDEF(TRACE, printf("[ALLOCATE_DIRECT:3] old buffer=%lx "
                            "top=%lx hard_end=%lx available=%zu words\n",
                            d->buffer, d->region_top, d->region_hard_end,
                            (d->region_hard_end - d->region_top) / 8));

        s->sub_stage = 4;
        break;
    }

    case 4:
    {
        if (d->region_top < d->region_hard_end)
        {
            size_t words = (d->region_hard_end - d->region_top) / 8;
            uintptr_t cur_klass = 0;
            uint header_words = d->pars.useCompressedKlassPointers ? 2 : 3;

            IFDEF(TRACE, printf("[ALLOCATE_DIRECT:4] fill abandoned PLAB "
                                "top=%lx hard_end=%lx words=%zu\n",
                                d->region_top, d->region_hard_end, words));

            if (words >= header_words)
            {
                uint payload_size = words - header_words;
                uint32_t len = payload_size * 2;
                uintptr_t array_len_addr = d->region_top +
                                           (d->pars.useCompressedKlassPointers ? 12 : 16);

                IFDEF(TRACE, printf("[ALLOCATE_DIRECT:4] create filler int[] "
                                    "len=%u len_addr=%lx\n",
                                    len, array_len_addr));

                if (!hwgc_access(s, array_len_addr, &len, 4, true))
                    return;

                cur_klass = d->pars.intArrayKlassObj;
            }
            else if (words > 0)
            {
                cur_klass = d->pars.objectKlass;
            }

            IFDEF(TRACE, printf("[ALLOCATE_DIRECT:4] filler klass=%lx\n",
                                cur_klass));

            uintptr_t mark = 0x1;
            if (!hwgc_access(s, d->region_top, &mark, 8, true))
                return;

            if (d->pars.useCompressedKlassPointers)
            {
                uint32_t narrow_klass = (uint32_t)((cur_klass -
                                                    s->stageData.pars.compressedKlassPointerBase) >>
                                                   s->stageData.pars.compressedKlassPointerShift);

                if (!hwgc_access(s, d->region_top + 0x8,
                                 &narrow_klass, 4, true))
                    return;
            }
            else
            {
                if (!hwgc_access(s, d->region_top + 0x8,
                                 &cur_klass, 8, true))
                    return;
            }
        }
        else
        {
            IFDEF(TRACE, printf("[ALLOCATE_DIRECT:4] old PLAB already empty\n"));
        }

        s->sub_stage = 5;
        break;
    }

    case 5:
    {
        d->min_word_size = d->required_in_plab;
        d->desired_word_size = d->plab_word_size;
        d->during_gc_select = 0;

        IFDEF(TRACE, printf("[ALLOCATE_DIRECT:5] goto ALLOCATE_DURING_GC "
                            "min=%zu desired=%zu select=%u\n",
                            d->min_word_size, d->desired_word_size,
                            d->during_gc_select));

        hwgc_goto_stage(s, STAGE_ALLOCATE_DURING_GC, 0);
        break;
    }

    case 6:
    {
        if (d->to_obj != 0)
        {
            uintptr_t write30 =
                (d->actual_plab_size - 2) >= d->size ? d->to_obj + d->size * 8 : d->to_obj;

            uintptr_t write38 =
                d->to_obj + (d->actual_plab_size - 2) * 8;
            uintptr_t write40 =
                d->to_obj + d->actual_plab_size * 8;
            uintptr_t write48;

            if (!hwgc_access(s, d->buffer + 0x48, &write48, 8, false))
                return;

            write48 += d->actual_plab_size;

            IFDEF(TRACE, printf("[ALLOCATE_DIRECT:6] install new PLAB "
                                "buffer=%lx to=%lx actual=%zu "
                                "bottom=%lx top=%lx end=%lx alloc_words=%lx\n",
                                d->buffer, d->to_obj, d->actual_plab_size,
                                d->to_obj, write30, write40, write48));

            if (!hwgc_access(s, d->buffer + 0x20,
                             &d->actual_plab_size, 8, true))
                return;
            if (!hwgc_access(s, d->buffer + 0x28, &d->to_obj, 8, true))
                return;
            if (!hwgc_access(s, d->buffer + 0x30, &write30, 8, true))
                return;
            if (!hwgc_access(s, d->buffer + 0x38, &write38, 8, true))
                return;
            if (!hwgc_access(s, d->buffer + 0x40, &write40, 8, true))
                return;
            if (!hwgc_access(s, d->buffer + 0x48, &write48, 8, true))
                return;

            IFDEF(TRACE, printf("[ALLOCATE_DIRECT:6] PLAB installed, "
                                "return previous stage\n"));

            if (d->actual_plab_size - 2 < d->size)
                d->to_obj = 0;

            hwgc_return_previous(s);
        }
        else if (d->region_top < d->region_hard_end)
        {
            IFDEF(TRACE, printf("[ALLOCATE_DIRECT:6] allocation failed, "
                                "retire remaining old PLAB [%lx, %lx)\n",
                                d->region_top, d->region_hard_end));

            if (!hwgc_access(s, d->buffer + 0x38,
                             &d->region_hard_end, 8, true))
                return;
            if (!hwgc_access(s, d->buffer + 0x30,
                             &d->region_hard_end, 8, true))
                return;
            if (!hwgc_access(s, d->buffer + 0x28,
                             &d->region_hard_end, 8, true))
                return;

            d->plab_refill_failed = true;
            s->sub_stage = 7;
        }
        else
        {
            IFDEF(TRACE, printf("[ALLOCATE_DIRECT:6] allocation failed, "
                                "old PLAB exhausted, request direct object "
                                "allocation\n"));

            s->sub_stage = 7;
        }
        break;
    }

    case 7:
    {
        d->min_word_size = d->size;
        d->desired_word_size = d->size;
        d->during_gc_select = 1;

        IFDEF(TRACE, printf("[ALLOCATE_DIRECT:7] direct allocation request "
                            "min=%zu desired=%zu select=%u\n",
                            d->min_word_size, d->desired_word_size,
                            d->during_gc_select));

        hwgc_goto_stage(s, STAGE_ALLOCATE_DURING_GC, 0);
        break;
    }

    case 8:
    {
        IFDEF(TRACE, printf("[ALLOCATE_DIRECT:8] return previous stage\n"));

        hwgc_return_previous(s);
        break;
    }

    default:
        IFDEF(TRACE, printf("[ALLOCATE_DIRECT] invalid sub=%u, reset\n",
                            s->sub_stage));

        hwgc_goto_stage(s, STAGE_ALLOCATE_DIRECT, 0);
        break;
    }
}

static void stage_allocate_during_gc_function(HWGCDevState *s)
{
    struct HWGCStageData *d = &s->stageData;
    int8_t dest_attr_type = (int8_t)(d->dest_attr >> 8);
    uint expected = 0;
    uint writed = 1;
    uint get;
    uintptr_t lock_ptr;

    IFDEF(TRACE, printf("[ALLOCATE_DURING_GC] sub=%u dest_attr=0x%x "
                        "dest_type=%d size=%zu to=%lx allocator=%lx\n",
                        s->sub_stage, d->dest_attr, dest_attr_type,
                        d->size, d->to_obj, d->allocator_ptr));

    switch (s->sub_stage)
    {
    case 0:
    {
        if (dest_attr_type == 0)
        {
            IFDEF(TRACE, printf("[ALLOCATE_DURING_GC:0] read young "
                                "region_ptr from allocator=%lx\n",
                                d->allocator_ptr + 0x28));

            if (!hwgc_access(s, d->allocator_ptr + 0x28,
                             &d->region_ptr, 8, false))
                return;
        }
        else
        {
            d->region_ptr = d->allocator_ptr + 0x30;
        }

        IFDEF(TRACE, printf("[ALLOCATE_DURING_GC:0] region_ptr=%lx\n",
                            d->region_ptr));

        s->sub_stage = 1;
        break;
    }

    case 1:
    {
        IFDEF(TRACE, printf("[ALLOCATE_DURING_GC:1] read alloc_region "
                            "from=%lx\n",
                            d->region_ptr + 0x8));

        if (!hwgc_access(s, d->region_ptr + 0x8,
                         &d->alloc_region, 8, false))
            return;

        IFDEF(TRACE, printf("[ALLOCATE_DURING_GC:1] alloc_region=%lx\n",
                            d->alloc_region));

        s->sub_stage = 2;
        break;
    }

    case 2:
    {
        if (dest_attr_type == 0)
        {
            d->par_alloc_iml_sel = 0;

            IFDEF(TRACE, printf("[ALLOCATE_DURING_GC:2] young destination, "
                                "goto PAR_ALLOCATE_IML sel=0\n"));

            hwgc_goto_stage(s, STAGE_PAR_ALLOCATE_IML, 0);
        }
        else
        {
            lock_ptr = d->alloc_region + 0x40;
            expected = 0;
            writed = 1;

            IFDEF(TRACE, printf("[ALLOCATE_DURING_GC:2] acquire old lock "
                                "addr=%lx expected=%u write=%u\n",
                                lock_ptr + 8, expected, writed));

            if (!hwgc_cmpxchg(s, lock_ptr + 8,
                              expected, writed, 4, &get))
                return;

            IFDEF(TRACE, printf("[ALLOCATE_DURING_GC:2] old lock CAS "
                                "returned=%u %s\n",
                                get, get == expected ? "ACQUIRED" : "BUSY"));

            if (get == expected)
            {
                d->par_alloc_sel = 0;
                d->bot_updates = true;

                IFDEF(TRACE, printf("[ALLOCATE_DURING_GC:2] goto "
                                    "PAR_ALLOCATE sel=0 bot_updates=1\n"));

                hwgc_goto_stage(s, STAGE_PAR_ALLOCATE, 0);
            }
        }
        break;
    }

    case 3:
    {
        lock_ptr = d->alloc_region + 0x40;
        expected = 1;
        writed = 0;

        IFDEF(TRACE, printf("[ALLOCATE_DURING_GC:3] release region lock "
                            "addr=%lx\n",
                            lock_ptr + 8));

        if (!hwgc_cmpxchg(s, lock_ptr + 8,
                          expected, writed, 4, &get))
            return;

        IFDEF(TRACE, printf("[ALLOCATE_DURING_GC:3] release result=%u\n",
                            get));

        if (get > 1)
        {
            IFDEF(TRACE, printf("[ALLOCATE_DURING_GC:3] lock has waiters, "
                                "raise IRQ_WAKE\n"));

            s->irq_par0 = lock_ptr;
            s->timer_running = false;
            s->irq_to_sub_stage = 4;
            qatomic_or(&s->status, ST_WAIT_WAKE);

            hwgc_raise_irq_from_worker(s, IRQ_WAKE);
        }
        else
        {
            s->sub_stage = 4;
        }
        break;
    }

    case 4:
    {
        if (d->to_obj == 0)
        {
            IFDEF(TRACE, printf("[ALLOCATE_DURING_GC:4] allocation failed, "
                                "read allocator full flag addr=%lx\n",
                                d->allocator_ptr + 0x10));

            if (!hwgc_access(s, d->allocator_ptr + 0x10,
                             &d->region_attr_ptr, 1, false))
                return;

            IFDEF(TRACE, printf("[ALLOCATE_DURING_GC:4] allocator flag=0x%lx\n",
                                d->region_attr_ptr));

            s->sub_stage = 6;
        }
        else
        {
            IFDEF(TRACE, printf("[ALLOCATE_DURING_GC:4] allocated to=%lx, "
                                "goto ALLOCATE_DIRECT\n",
                                d->to_obj));

            s->sub_stage = 5;
        }
        break;
    }

    case 5:
    {
        IFDEF(TRACE, printf("[ALLOCATE_DURING_GC:5] goto ALLOCATE_DIRECT "
                            "sub=%u, to=%lx\n",
                            d->during_gc_select ? 8 : 6, d->to_obj));

        hwgc_goto_stage(s, STAGE_ALLOCATE_DIRECT,
                        d->during_gc_select ? 8 : 6);
        break;
    }

    case 6:
    {
        bool is_full = dest_attr_type == 0 ? (d->region_attr_ptr & 0x1) != 0 : (d->region_attr_ptr & 0x2) != 0;

        IFDEF(TRACE, printf("[ALLOCATE_DURING_GC:6] allocator flag=0x%lx "
                            "dest_type=%d is_full=%d\n",
                            d->region_attr_ptr, dest_attr_type, is_full));

        if (!is_full)
        {
            if (dest_attr_type == 0)
            {
                d->par_alloc_iml_sel = 1;

                IFDEF(TRACE, printf("[ALLOCATE_DURING_GC:6] retry young "
                                    "PAR_ALLOCATE_IML sel=1\n"));

                hwgc_goto_stage(s, STAGE_PAR_ALLOCATE_IML, 0);
            }
            else
            {
                lock_ptr = d->alloc_region + 0x40;
                expected = 0;
                writed = 1;

                IFDEF(TRACE, printf("[ALLOCATE_DURING_GC:6] retry old lock "
                                    "addr=%lx\n",
                                    lock_ptr + 8));

                if (!hwgc_cmpxchg(s, lock_ptr + 8,
                                  expected, writed, 4, &get))
                    return;

                IFDEF(TRACE, printf("[ALLOCATE_DURING_GC:6] retry lock CAS "
                                    "returned=%u %s\n",
                                    get,
                                    get == expected ? "ACQUIRED" : "BUSY"));

                if (get == expected)
                {
                    d->par_alloc_sel = 1;
                    d->bot_updates = true;

                    IFDEF(TRACE, printf("[ALLOCATE_DURING_GC:6] goto "
                                        "PAR_ALLOCATE sel=1 bot_updates=1\n"));

                    hwgc_goto_stage(s, STAGE_PAR_ALLOCATE, 0);
                }
            }
        }
        else
        {
            IFDEF(TRACE, printf("[ALLOCATE_DURING_GC:6] allocator full, "
                                "goto ALLOCATE_DIRECT\n"));

            s->sub_stage = 5;
        }
        break;
    }

    case 7:
    {
        lock_ptr = d->alloc_region + 0x40;
        expected = 1;
        writed = 0;

        IFDEF(TRACE, printf("[ALLOCATE_DURING_GC:7] release retry lock "
                            "addr=%lx\n",
                            lock_ptr + 8));

        if (!hwgc_cmpxchg(s, lock_ptr + 8,
                          expected, writed, 4, &get))
            return;

        IFDEF(TRACE, printf("[ALLOCATE_DURING_GC:7] release result=%u\n",
                            get));

        if (get > 1)
        {
            IFDEF(TRACE, printf("[ALLOCATE_DURING_GC:7] lock has waiters, "
                                "raise IRQ_WAKE\n"));

            s->irq_par0 = lock_ptr;
            s->timer_running = false;
            s->irq_to_sub_stage = 8;
            qatomic_or(&s->status, ST_WAIT_WAKE);

            hwgc_raise_irq_from_worker(s, IRQ_WAKE);
        }
        else
        {
            s->sub_stage = 8;
        }
        break;
    }

    case 8:
    {
        if (d->to_obj == 0)
        {
            expected = 0;
            writed = 1;

            if (!hwgc_cmpxchg(s, d->pars.lockPtr + 8,
                              expected, writed, 4, &get))
                return;

            IFDEF(TRACE, printf("[ALLOCATE_DURING_GC:8] global lock CAS "
                                "returned=%u %s\n",
                                get,
                                get == expected ? "ACQUIRED" : "BUSY"));

            if (get == expected)
                s->sub_stage = 9;
        }
        else
        {
            IFDEF(TRACE, printf("[ALLOCATE_DURING_GC:8] allocation succeeded "
                                "to=%lx, goto ALLOCATE_DIRECT\n",
                                d->to_obj));

            s->sub_stage = 5;
        }
        break;
    }

    case 9:
    {
        IFDEF(TRACE, printf("[ALLOCATE_DURING_GC:9] set allocation thread "
                            "thread=%lx lock_base=%lx, goto ATTEMPT_ALLOC\n",
                            d->pars.thread, d->pars.lockPtr));

        if (!hwgc_access(s, d->pars.lockPtr,
                         &d->pars.thread, 8, true))
            return;

        hwgc_goto_stage(s, STAGE_ATTEMPT_ALLOC, 0);
        assert(true);
        break;
    }

    case 10:
    {
        if (d->to_obj == 0)
        {
            uintptr_t addr = dest_attr_type == 0 ? d->allocator_ptr + 0x10 : d->allocator_ptr + 0x11;
            uint8_t full = 1;

            IFDEF(TRACE, printf("[ALLOCATE_DURING_GC:10] mark allocator full "
                                "addr=%lx dest_type=%d\n",
                                addr, dest_attr_type));

            if (!hwgc_access(s, addr, &full, 1, true))
                return;
        }

        s->sub_stage = 11;
        break;
    }

    case 11:
    {
        expected = 1;
        writed = 0;

        if (!hwgc_cmpxchg(s, d->pars.lockPtr + 8,
                          expected, writed, 4, &get))
            return;

        IFDEF(TRACE, printf("[ALLOCATE_DURING_GC:11] global unlock result=%u\n",
                            get));

        if (get > 1)
        {
            IFDEF(TRACE, printf("[ALLOCATE_DURING_GC:11] global lock waiters, "
                                "raise IRQ_WAKE\n"));

            s->irq_par0 = d->pars.lockPtr + 8;
            s->timer_running = false;
            s->irq_to_sub_stage = 5;
            qatomic_or(&s->status, ST_WAIT_WAKE);

            hwgc_raise_irq_from_worker(s, IRQ_WAKE);
        }
        else
        {
            s->sub_stage = 5;
        }
        break;
    }

    default:
        IFDEF(TRACE, printf("[ALLOCATE_DURING_GC] invalid sub=%u, reset\n",
                            s->sub_stage));

        hwgc_goto_stage(s, STAGE_ALLOCATE_DURING_GC, 0);
        break;
    }
}

static void stage_par_allocate_iml_function(HWGCDevState *s)
{
    struct HWGCStageData *d = &s->stageData;

    IFDEF(TRACE, printf("[PAR_ALLOCATE_IML] sub=%u sel=%u alloc_region=%lx "
                        "top=%lx end=%lx min=%zu desired=%zu to=%lx\n",
                        s->sub_stage, d->par_alloc_iml_sel,
                        d->alloc_region, d->alloc_top, d->alloc_end,
                        d->min_word_size, d->desired_word_size,
                        d->to_obj));

    switch (s->sub_stage)
    {
    case 0:
    {
        IFDEF(TRACE, printf("[PAR_ALLOCATE_IML:0] read alloc top/end "
                            "region=%lx top_addr=%lx end_addr=%lx\n",
                            d->alloc_region,
                            d->alloc_region + 0x10,
                            d->alloc_region + 0x8));

        if (!hwgc_access(s, d->alloc_region + 0x10,
                         &d->alloc_top, 8, false))
            return;

        if (!hwgc_access(s, d->alloc_region + 0x8,
                         &d->alloc_end, 8, false))
            return;

        IFDEF(TRACE, printf("[PAR_ALLOCATE_IML:0] alloc_top=%lx "
                            "alloc_end=%lx\n",
                            d->alloc_top, d->alloc_end));

        s->sub_stage = 1;
        break;
    }

    case 1:
    {
        size_t available = (d->alloc_end - d->alloc_top) / 8;
        d->want_to_allocate = MIN(available, d->desired_word_size);

        IFDEF(TRACE, printf("[PAR_ALLOCATE_IML:1] available=%zu words "
                            "want=%zu min=%zu desired=%zu\n",
                            available, d->want_to_allocate,
                            d->min_word_size, d->desired_word_size));

        if (d->want_to_allocate >= d->min_word_size)
        {
            uintptr_t new_top = d->alloc_top + d->want_to_allocate * 8;

            IFDEF(TRACE, printf("[PAR_ALLOCATE_IML:1] CAS top: addr=%lx "
                                "old=%lx new=%lx\n",
                                d->alloc_region + 0x10,
                                d->alloc_top, new_top));

            if (!hwgc_cmpxchg(s, d->alloc_region + 0x10,
                              d->alloc_top, new_top, 8,
                              &d->region_attr_ptr))
                return;

            IFDEF(TRACE, printf("[PAR_ALLOCATE_IML:1] CAS returned=%lx %s\n",
                                d->region_attr_ptr,
                                d->region_attr_ptr == d->alloc_top ? "SUCCESS" : "RETRY"));

            if (d->region_attr_ptr == d->alloc_top)
            {
                d->actual_plab_size = d->want_to_allocate;
                d->to_obj = d->alloc_top;

                IFDEF(TRACE, printf("[PAR_ALLOCATE_IML:1] allocated "
                                    "to=%lx actual_plab_size=%zu\n",
                                    d->to_obj, d->actual_plab_size));

                s->sub_stage = 2;
            }
            else
            {
                s->sub_stage = 0;
            }
        }
        else
        {
            d->actual_plab_size = 0;
            d->to_obj = 0;

            IFDEF(TRACE, printf("[PAR_ALLOCATE_IML:1] insufficient space: "
                                "available=%zu want=%zu min=%zu\n",
                                available, d->want_to_allocate,
                                d->min_word_size));

            s->sub_stage = 2;
        }
        break;
    }

    case 2:
    {
        IFDEF(TRACE, printf("[PAR_ALLOCATE_IML:2] result to=%lx "
                            "actual_plab_size=%zu sel=%u\n",
                            d->to_obj, d->actual_plab_size,
                            d->par_alloc_iml_sel));

        if (d->par_alloc_iml_sel == 2)
        {
            IFDEF(TRACE, printf("[PAR_ALLOCATE_IML:2] goto PAR_ALLOCATE:1\n"));

            hwgc_goto_stage(s, STAGE_PAR_ALLOCATE, 1);
        }
        else if (d->par_alloc_iml_sel == 1)
        {
            IFDEF(TRACE, printf("[PAR_ALLOCATE_IML:2] goto "
                                "ALLOCATE_DURING_GC:8\n"));

            hwgc_goto_stage(s, STAGE_ALLOCATE_DURING_GC, 8);
        }
        else
        {
            IFDEF(TRACE, printf("[PAR_ALLOCATE_IML:2] goto "
                                "ALLOCATE_DURING_GC:4\n"));

            hwgc_goto_stage(s, STAGE_ALLOCATE_DURING_GC, 4);
        }
        break;
    }

    default:
        IFDEF(TRACE, printf("[PAR_ALLOCATE_IML] invalid sub=%u, reset\n",
                            s->sub_stage));

        hwgc_goto_stage(s, STAGE_PAR_ALLOCATE_IML, 0);
        break;
    }
}

static void stage_par_allocate_function(HWGCDevState *s)
{
    struct HWGCStageData *d = &s->stageData;

    IFDEF(TRACE, printf("[PAR_ALLOCATE] sub=%u sel=%u to=%lx "
                        "actual_plab=%zu alloc_region=%lx bot_updates=%d\n",
                        s->sub_stage, d->par_alloc_sel, d->to_obj,
                        d->actual_plab_size, d->alloc_region,
                        d->bot_updates));

    switch (s->sub_stage)
    {
    case 0:
    {
        d->par_alloc_iml_sel = 2;

        IFDEF(TRACE, printf("[PAR_ALLOCATE:0] goto PAR_ALLOCATE_IML "
                            "sel=2\n"));

        hwgc_goto_stage(s, STAGE_PAR_ALLOCATE_IML, 0);
        return;
    }

    case 1:
    {
        if (d->to_obj != 0 && d->bot_updates)
        {
            d->blk_start = d->to_obj;
            d->blk_end = d->to_obj + d->actual_plab_size * 8;
            d->bot_part_ptr = d->alloc_region + 0x20;

            IFDEF(TRACE, printf("[PAR_ALLOCATE:1] BOT update: "
                                "block=[%lx, %lx) bot_part=%lx\n",
                                d->blk_start, d->blk_end,
                                d->bot_part_ptr));

            if (!hwgc_access(s, d->bot_part_ptr,
                             &d->next_offset_threshold, 8, false))
                return;

            IFDEF(TRACE, printf("[PAR_ALLOCATE:1] "
                                "next_offset_threshold=%lx\n",
                                d->next_offset_threshold));

            s->sub_stage = 2;
        }
        else
        {
            IFDEF(TRACE, printf("[PAR_ALLOCATE:1] skip BOT update: "
                                "to=%lx bot_updates=%d\n",
                                d->to_obj, d->bot_updates));

            s->sub_stage = 9;
        }
        break;
    }

    case 2:
    {
        IFDEF(TRACE, printf("[PAR_ALLOCATE:2] blk_end=%lx "
                            "next_threshold=%lx\n",
                            d->blk_end, d->next_offset_threshold));

        if (d->blk_end > d->next_offset_threshold)
        {
            if (!hwgc_access(s, d->bot_part_ptr + 0x8,
                             &d->index, 8, false))
                return;

            if (!hwgc_access(s, d->bot_part_ptr + 0x10,
                             &d->bot_ptr, 8, false))
                return;

            IFDEF(TRACE, printf("[PAR_ALLOCATE:2] BOT index=%zu "
                                "bot_ptr=%lx\n",
                                d->index, d->bot_ptr));

            s->sub_stage = 3;
        }
        else
        {
            IFDEF(TRACE, printf("[PAR_ALLOCATE:2] block does not cross "
                                "next BOT threshold, skip update\n"));

            s->sub_stage = 9;
        }
        break;
    }

    case 3:
    {
        IFDEF(TRACE, printf("[PAR_ALLOCATE:3] read BOT array ptr "
                            "from=%lx\n",
                            d->bot_ptr + 0x10));

        if (!hwgc_access(s, d->bot_ptr + 0x10,
                         &d->array, 8, false))
            return;

        IFDEF(TRACE, printf("[PAR_ALLOCATE:3] BOT array=%lx\n",
                            d->array));

        s->sub_stage = 4;
        break;
    }

    case 4:
    {
        uint8_t value = (uint8_t)((d->next_offset_threshold - d->blk_start) / 8);
        if (!hwgc_access(s, d->array + d->index, &value, 1, true))
            return;

        s->sub_stage = 5;
        break;
    }

    case 5:
    {
        if (!hwgc_access(s, d->bot_ptr,
                         &d->reserved_start, 8, false))
            return;

        IFDEF(TRACE, printf("[PAR_ALLOCATE:5] reserved_start=%lx\n",
                            d->reserved_start));

        s->sub_stage = 6;
        break;
    }

    case 6:
    {
        size_t end_index = (d->blk_end - 8 - d->reserved_start) >> 9;
        uintptr_t rem_st = d->reserved_start + ((d->index + 1) << 6) * 8;
        uintptr_t rem_end = d->reserved_start + ((end_index << 6) + 64) * 8;

        d->start_card = (rem_st - d->reserved_start) >> 9;
        d->end_card = (rem_end - 8 - d->reserved_start) >> 9;

        IFDEF(TRACE, printf("[PAR_ALLOCATE:6] end_index=%zu rem=[%lx, %lx) "
                            "card=[%zu, %zu]\n",
                            end_index, rem_st, rem_end,
                            d->start_card, d->end_card));

        if (d->index + 1 <= end_index &&
            rem_st < rem_end &&
            d->start_card <= d->end_card)
        {
            d->remaining = d->end_card - d->start_card + 1;
            d->begin = d->array + d->start_card;
            d->i = 0;

            IFDEF(TRACE, printf("[PAR_ALLOCATE:6] batch fill begin=%lx "
                                "remaining=%zu reset_i=%u\n",
                                d->begin, d->remaining, d->i));

            s->sub_stage = 7;
        }
        else
        {
            IFDEF(TRACE, printf("[PAR_ALLOCATE:6] no remaining BOT cards "
                                "to update\n"));

            s->sub_stage = 8;
        }
        break;
    }

    case 7:
    {
        if (d->i < 14 && d->remaining > 0)
        {
            size_t chunk = (size_t)15 << (4 * d->i);
            size_t nbytes = MIN(d->remaining, chunk);
            uint8_t offset = (uint8_t)(64 + d->i);

            IFDEF(TRACE, printf("[PAR_ALLOCATE:7] fill i=%u offset=%u "
                                "begin=%lx bytes=%zu remaining=%zu\n",
                                d->i, offset, d->begin,
                                nbytes, d->remaining));

            for (int j = 0; j < nbytes; ++j)
            {
                if (!hwgc_access(s, d->begin, &offset, 1, true))
                    return;
                d->begin = d->begin + 1;
                d->remaining = d->remaining - 1;
            }

            d->i++;
        }
        else
        {
            IFDEF(TRACE, printf("[PAR_ALLOCATE:7] batch fill complete: "
                                "i=%u remaining=%zu\n",
                                d->i, d->remaining));

            s->sub_stage = 8;
        }
        break;
    }

    case 8:
    {
        size_t end_index =
            (d->blk_end - 8 - d->reserved_start) >> 9;

        d->index = end_index + 1;
        d->next_offset_threshold = d->reserved_start +
                                   ((end_index << 6) + 64) * 8;

        IFDEF(TRACE, printf("[PAR_ALLOCATE:8] update BOT metadata: "
                            "index=%zu next_threshold=%lx\n",
                            d->index, d->next_offset_threshold));

        if (!hwgc_access(s, d->bot_part_ptr,
                         &d->next_offset_threshold, 8, true))
            return;

        if (!hwgc_access(s, d->bot_part_ptr + 0x8,
                         &d->index, 8, true))
            return;

        s->sub_stage = 9;
        break;
    }

    case 9:
    {
        IFDEF(TRACE, printf("[PAR_ALLOCATE:9] return route "
                            "par_alloc_sel=%u to=%lx\n",
                            d->par_alloc_sel, d->to_obj));

        if (d->par_alloc_sel == 2)
        {
            IFDEF(TRACE, printf("[PAR_ALLOCATE:9] "
                                "goto ATTEMPT_ALLOC:17\n"));

            hwgc_goto_stage(s, STAGE_ATTEMPT_ALLOC, 17);
        }
        else if (d->par_alloc_sel == 1)
        {
            IFDEF(TRACE, printf("[PAR_ALLOCATE:9] "
                                "goto ALLOCATE_DURING_GC:7\n"));

            hwgc_goto_stage(s, STAGE_ALLOCATE_DURING_GC, 7);
        }
        else
        {
            IFDEF(TRACE, printf("[PAR_ALLOCATE:9] "
                                "goto ALLOCATE_DURING_GC:3\n"));

            hwgc_goto_stage(s, STAGE_ALLOCATE_DURING_GC, 3);
        }
        break;
    }

    default:
        IFDEF(TRACE, printf("[PAR_ALLOCATE] invalid sub=%u, reset\n",
                            s->sub_stage));

        hwgc_goto_stage(s, STAGE_PAR_ALLOCATE, 0);
        break;
    }
}

static void stage_trace_function(HWGCDevState *s)
{
    struct HWGCStageData *d = &s->stageData;

    IFDEF(TRACE, printf("[TRACE] sub=%u from=%lx to=%lx lh=%d kid=%d "
                        "array_len=%u end=%u scanning_young=%d\n",
                        s->sub_stage, d->from_obj, d->to_obj,
                        d->lh, d->kid, d->common_oop_array_length,
                        d->end, d->scanning_in_young));

    switch (s->sub_stage)
    {
    case 0:
    {
        d->scanning_in_young = (int8_t)(d->dest_attr >> 8) == 0;

        IFDEF(TRACE, printf("[TRACE:0] dest_attr=0x%x scanning_young=%d "
                            "lh=%d\n",
                            d->dest_attr, d->scanning_in_young, d->lh));

        if (d->lh < 0)
        {
            uintptr_t len_addr;

            d->end = d->common_oop_array_length % d->pars.chunkSize;
            len_addr = d->to_obj +
                       (d->pars.useCompressedKlassPointers ? 12 : 16);

            IFDEF(TRACE, printf("[TRACE:0] array trace: src_len=%u "
                                "chunk=%u first_chunk_len=%u "
                                "dst_len_addr=%lx\n",
                                d->common_oop_array_length,
                                (unsigned)d->pars.chunkSize,
                                d->end, len_addr));

            if (!hwgc_access(s, len_addr, &d->end, 4, true))
                return;

            s->sub_stage = 1;
        }
        else
        {
            IFDEF(TRACE, printf("[TRACE:0] instance trace: read vtable_len "
                                "addr=%lx\n",
                                d->klass_ptr + 160));

            if (!hwgc_access(s, d->klass_ptr + 160,
                             &d->vtable_len, 4, false))
                return;

            s->sub_stage = 3;
        }
        break;
    }

    case 1:
    {
        if (d->common_oop_array_length > d->end)
        {
            uintptr_t pushData = d->from_obj + 0x2;
            uintptr_t queue_addr =
                d->pars.taskQueueElemsBase + d->localBot * 8;

            IFDEF(TRACE, printf("[TRACE:1] push remaining array task=%lx "
                                "queue[%u]=%lx\n",
                                pushData, d->localBot, queue_addr));

            if (!hwgc_access(s, queue_addr, &pushData, 8, true))
                return;

            d->localBot = (d->localBot + 1) & ((1 << 17) - 1);

            IFDEF(TRACE, printf("[TRACE:1] pushed, next localBot=%u\n",
                                d->localBot));
        }
        else
        {
            IFDEF(TRACE, printf("[TRACE:1] array fully handled in "
                                "current chunk, no extra task\n"));
        }

        s->sub_stage = 2;
        break;
    }

    case 2:
    {
        size_t oop_size = d->pars.useCompressedOops ? 4 : 8;
        uintptr_t base = d->to_obj +
                         (d->pars.useCompressedKlassPointers ? 16 : 24);
        uintptr_t low = base;
        uintptr_t high = base + d->end * oop_size;

        d->p = base;
        d->q = base + d->common_oop_array_length * oop_size;

        if (d->p < low)
            d->p = low;

        if (d->q > high)
            d->q = high;

        IFDEF(TRACE, printf("[TRACE:2] array oop scan: base=%lx "
                            "oop_size=%zu p=%lx q=%lx count=%zu\n",
                            base, oop_size, d->p, d->q,
                            (d->q - d->p) / oop_size));

        d->previous = STAGE_TRACE_PLUS;
        d->previous_sub_stage = 0;

        d->done_to = STAGE_COMMON_OOP;
        d->doneto_sub_stage = 4;

        IFDEF(TRACE, printf("[TRACE:2] goto TRACE_PLUS, "
                            "done_to=COMMON_OOP:4\n"));

        hwgc_goto_stage(s, STAGE_TRACE_PLUS, 0);
        break;
    }

    case 3:
    {
        IFDEF(TRACE, printf("[TRACE:3] vtable_len=%u, read oop-map info "
                            "addr=%lx\n",
                            d->vtable_len, d->klass_ptr + 296));

        if (!hwgc_access(s, d->klass_ptr + 296,
                         &d->region_attr_ptr, 8, false))
            return;

        IFDEF(TRACE, printf("[TRACE:3] oop-map info raw=%lx\n",
                            d->region_attr_ptr));

        s->sub_stage = 4;
        break;
    }

    case 4:
    {
        int itable_len = d->region_attr_ptr >> 32;
        int nonStaticOopMapSize = (int)d->region_attr_ptr;

        d->start_map = (uintptr_t)((uintptr_t *)(d->klass_ptr + 464) +
                                   d->vtable_len + itable_len);
        d->end_map = d->start_map + nonStaticOopMapSize * 8;

        IFDEF(TRACE, printf("[TRACE:4] itable_len=%d oop_map_size=%d "
                            "map=[%lx, %lx)\n",
                            itable_len, nonStaticOopMapSize,
                            d->start_map, d->end_map));

        s->sub_stage = 5;
        break;
    }

    case 5:
    {
        if (d->start_map < d->end_map)
        {
            uintptr_t read_addr = d->end_map - 8;
            if (!hwgc_access(s, read_addr, &d->region_attr_ptr, 8, false))
                return;

            d->end_map = read_addr;

            IFDEF(TRACE, printf("[TRACE:5] oop-map entry raw=%lx\n",
                                d->region_attr_ptr));

            s->sub_stage = 6;
        }
        else
        {
            IFDEF(TRACE, printf("[TRACE:5] oop-map scan complete, "
                                "process special kid=%d\n",
                                d->kid));

            s->sub_stage = 7;
        }
        break;
    }

    case 6:
    {
        int offset = (int)d->region_attr_ptr;
        int count = d->region_attr_ptr >> 32;
        size_t oop_size = d->pars.useCompressedOops ? 4 : 8;

        d->p = d->to_obj + offset;
        d->q = d->p + count * oop_size;

        IFDEF(TRACE, printf("[TRACE:6] oop-map range: offset=%d count=%d "
                            "p=%lx q=%lx\n",
                            offset, count, d->p, d->q));

        d->previous = STAGE_TRACE_DEC;
        d->previous_sub_stage = 0;

        d->done_to = STAGE_TRACE;
        d->doneto_sub_stage = 5;

        IFDEF(TRACE, printf("[TRACE:6] goto TRACE_DEC, "
                            "return_to=TRACE:5\n"));

        hwgc_goto_stage(s, STAGE_TRACE_DEC, 0);
        break;
    }

    case 7:
    {
        IFDEF(TRACE, printf("[TRACE:7] special class handling kid=%d\n",
                            d->kid));

        if (d->kid == 2)
        {
            uint count_offset = d->pars.useCompressedKlassPointers ? 0x28 : 0x24;
            IFDEF(TRACE, printf("[TRACE:7] read static count addr=%lx\n",
                                d->from_obj + count_offset));

            if (!hwgc_access(s, d->from_obj + count_offset,
                             &d->staticCount, 4, false))
                return;

            IFDEF(TRACE, printf("[TRACE:7] staticCount=%u\n",
                                d->staticCount));

            s->sub_stage = 8;
        }
        else if (d->kid == 1)
        {
            d->i = 0;

            IFDEF(TRACE, printf("[TRACE:7] reference-like object, "
                                "start special oop processing\n"));

            s->sub_stage = 9;
        }
        else
        {
            IFDEF(TRACE, printf("[TRACE:7] no special fields, "
                                "goto COMMON_OOP:4\n"));

            hwgc_goto_stage(s, STAGE_COMMON_OOP, 4);
        }
        break;
    }

    case 8:
    {
        size_t oop_size = d->pars.useCompressedOops ? 4 : 8;
        uint offset_of_static_fields = d->pars.useCompressedKlassPointers ? 0x70 : 0xb8;
        d->p = d->to_obj + offset_of_static_fields;
        d->q = d->p + d->staticCount * oop_size;

        IFDEF(TRACE, printf("[TRACE:8] static oop scan: p=%lx q=%lx "
                            "count=%u oop_size=%zu\n",
                            d->p, d->q, d->staticCount, oop_size));

        d->previous = STAGE_TRACE_PLUS;
        d->previous_sub_stage = 0;

        d->done_to = STAGE_COMMON_OOP;
        d->doneto_sub_stage = 4;

        IFDEF(TRACE, printf("[TRACE:8] goto TRACE_PLUS, "
                            "done_to=COMMON_OOP:4\n"));

        hwgc_goto_stage(s, STAGE_TRACE_PLUS, 0);
        break;
    }

    case 9:
    {
        if (d->i != 3)
        {
            uint32_t discovered_offset;
            uint32_t referent_offset;

            if (d->pars.useCompressedKlassPointers &&
                d->pars.useCompressedOops)
            {
                discovered_offset = 0x18;
                referent_offset = 0xc;
            }
            else if (d->pars.useCompressedOops)
            {
                discovered_offset = 0x1c;
                referent_offset = 0x10;
            }
            else
            {
                discovered_offset = 0x28;
                referent_offset = 0x10;
            }

            d->src = d->i == 1 ? d->from_obj + referent_offset : d->from_obj + discovered_offset;

            d->dest = d->i == 1 ? d->to_obj + referent_offset : d->to_obj + discovered_offset;

            IFDEF(TRACE, printf("[TRACE:9] special oop i=%u src=%lx dest=%lx "
                                "discovered_off=0x%x referent_off=0x%x\n",
                                d->i, d->src, d->dest,
                                discovered_offset, referent_offset));

            d->i++;
            d->previous = STAGE_TRACE;
            d->previous_sub_stage = 9;

            hwgc_goto_stage(s, STAGE_DO_OOP_WORK, 0);
        }
        else
        {
            d->i = 0;

            IFDEF(TRACE, printf("[TRACE:9] special oop scan complete, "
                                "goto COMMON_OOP:4\n"));

            hwgc_goto_stage(s, STAGE_COMMON_OOP, 4);
        }
        break;
    }

    default:
        IFDEF(TRACE, printf("[TRACE] invalid sub=%u, reset\n",
                            s->sub_stage));

        hwgc_goto_stage(s, STAGE_TRACE, 0);
        break;
    }
}

static void stage_copy_function(HWGCDevState *s)
{
    struct HWGCStageData *d = &s->stageData;

    switch (s->sub_stage)
    {
    case 0:
    {
        if (d->i >= d->size)
        {
            IFDEF(TRACE, printf("[COPY] complete: from=%lx to=%lx "
                                "copied_words=%zu\n",
                                d->from_obj, d->to_obj, d->size));

            hwgc_goto_stage(s, STAGE_COPY2SURVIVOR, 12);
            return;
        }

        /*
         * 不逐 word 打印，避免复制大对象时日志爆炸。
         * 打印首个、末个及每 256 个 word 的进度。
         */
        if ((d->i & 0xff) == 1 || d->i + 1 == d->size)
        {
            IFDEF(TRACE, printf("[COPY:0] progress=%u/%lu "
                                "src=%lx dst=%lx\n",
                                d->i, d->size,
                                d->from_obj + d->i * 8,
                                d->to_obj + d->i * 8));
        }

        /*
         * region_attr_ptr 用作异步 hwgc_access 的持久化读缓冲区，
         * 不宜改为局部变量。
         */
        if (!hwgc_access(s, d->from_obj + d->i * 8,
                         &d->region_attr_ptr, 8, false))
            return;

        if ((d->i & 0xff) == 1 || d->i + 1 == d->size)
        {
            IFDEF(TRACE, printf("[COPY:0] read word[%u]=%lx\n",
                                d->i, d->region_attr_ptr));
        }

        s->sub_stage = 1;
        break;
    }

    case 1:
    {
        if ((d->i & 0xff) == 0 || d->i + 1 == d->size)
        {
            IFDEF(TRACE, printf("[COPY:1] write word[%u]=%lx to=%lx\n",
                                d->i, d->region_attr_ptr,
                                d->to_obj + d->i * 8));
        }

        if (!hwgc_access(s, d->to_obj + d->i * 8,
                         &d->region_attr_ptr, 8, true))
            return;

        d->i++;
        s->sub_stage = 0;
        break;
    }

    default:
        IFDEF(TRACE, printf("[COPY] invalid sub=%u, reset\n",
                            s->sub_stage));

        hwgc_goto_stage(s, STAGE_COPY, 0);
        break;
    }
}

static void stage_trace_plus_function(HWGCDevState *s)
{
    struct HWGCStageData *d = &s->stageData;
    size_t oop_size = d->pars.useCompressedOops ? 4 : 8;

    if (d->p < d->q)
    {
        d->src = d->p - d->to_obj + d->from_obj;
        d->dest = d->p;
        d->p += oop_size;

        hwgc_goto_stage(s, STAGE_DO_OOP_WORK, 0);
    }
    else
        hwgc_return_done(s);
}

static void stage_trace_dec_function(HWGCDevState *s)
{
    struct HWGCStageData *d = &s->stageData;
    size_t oop_size = d->pars.useCompressedOops ? 4 : 8;

    if (d->p < d->q)
    {
        d->q -= oop_size;

        d->src = d->q - d->to_obj + d->from_obj;
        d->dest = d->q;

        hwgc_goto_stage(s, STAGE_DO_OOP_WORK, 0);
    }
    else
        hwgc_return_done(s);
}

static void stage_do_oop_work_function(HWGCDevState *s)
{
    struct HWGCStageData *d = &s->stageData;

    IFDEF(TRACE, printf("[DO_OOP_WORK] sub=%u src=%lx dest=%lx "
                        "heap_oop=%lx localBot=%u young=%d\n",
                        s->sub_stage, d->src, d->dest,
                        d->heap_oop, d->localBot,
                        d->scanning_in_young));

    switch (s->sub_stage)
    {
    case 0:
    {
        size_t oop_size = d->pars.useCompressedOops ? 4 : 8;

        IFDEF(TRACE, printf("[DO_OOP_WORK:0] read oop src=%lx size=%zu\n",
                            d->src, oop_size));

        if (!hwgc_access(s, d->src, &d->heap_oop, oop_size, false))
            return;

        IFDEF(TRACE, printf("[DO_OOP_WORK:0] raw oop=%lx\n",
                            d->heap_oop));

        s->sub_stage = 1;
        break;
    }

    case 1:
    {
        uintptr_t region_attr_addr;

        if (d->pars.useCompressedOops)
            d->heap_oop = (uint32_t)d->heap_oop;

        if (d->heap_oop == 0)
        {
            IFDEF(TRACE, printf("[DO_OOP_WORK:1] null oop, return previous\n"));

            s->sub_stage = 7;
            break;
        }

        if (d->pars.useCompressedOops)
        {
            d->heap_oop = d->pars.compressedOopBase +
                          (d->heap_oop << d->pars.compressedOopShift);
        }

        region_attr_addr = d->pars.regionAttrBiasedBase +
                           (d->heap_oop >> d->pars.regionAttrShiftBy) * 2;

        IFDEF(TRACE, printf("[DO_OOP_WORK:1] decoded oop=%lx "
                            "region_attr_addr=%lx\n",
                            d->heap_oop, region_attr_addr));

        if (!hwgc_access(s, region_attr_addr,
                         &d->region_attr_ptr, 2, false))
            return;

        IFDEF(TRACE, printf("[DO_OOP_WORK:1] region_attr=0x%x type=%d\n",
                            (uint16_t)d->region_attr_ptr,
                            (int8_t)(d->region_attr_ptr >> 8)));

        s->sub_stage = 2;
        break;
    }

    case 2:
    {
        int8_t region_attr_type =
            (int8_t)(d->region_attr_ptr >> 8);
        bool cross_region =
            ((d->dest ^ d->heap_oop) >>
             d->pars.logOfHRGrainBytes) != 0;

        IFDEF(TRACE, printf("[DO_OOP_WORK:2] attr=0x%x type=%d "
                            "cross_region=%d\n",
                            (uint16_t)d->region_attr_ptr,
                            region_attr_type, cross_region));

        if (region_attr_type >= 0)
        {
            uintptr_t writeElems = d->dest +
                                   (d->pars.useCompressedOops ? 1 : 0);
            uintptr_t queue_addr =
                d->pars.taskQueueElemsBase + d->localBot * 8;

            IFDEF(TRACE, printf("[DO_OOP_WORK:2] enqueue oop slot=%lx "
                                "queue[%u]=%lx\n",
                                writeElems, d->localBot, queue_addr));

            if (!hwgc_access(s, queue_addr, &writeElems, 8, true))
                return;

            d->localBot = (d->localBot + 1) & ((1 << 17) - 1);
            s->sub_stage = 7;
        }
        else if (cross_region)
        {
            if (region_attr_type == -2)
            {
                IFDEF(TRACE, printf("[DO_OOP_WORK:2] humongous candidate\n"));
                s->sub_stage = 3;
            }
            else
            {
                IFDEF(TRACE, printf("[DO_OOP_WORK:2] old/cross-region "
                                    "reference, process AOP\n"));
                s->sub_stage = 6;
            }
        }
        else
        {
            IFDEF(TRACE, printf("[DO_OOP_WORK:2] same region, return previous\n"));
            s->sub_stage = 7;
        }
        break;
    }

    case 3:
    {
        d->region = (d->heap_oop -
                     ((uintptr_t)d->pars.heapRegionBias << d->pars.heapRegionShiftBy)) >>
                    d->pars.logOfHRGrainBytes;

        IFDEF(TRACE, printf("[DO_OOP_WORK:3] humongous region=%u "
                            "candidate_addr=%lx\n",
                            d->region,
                            d->pars.humogousReclaimCandidateBoolBase +
                                d->region));

        if (!hwgc_access(s,
                         d->pars.humogousReclaimCandidateBoolBase + d->region,
                         &d->bool_base_value, 1, false))
            return;

        IFDEF(TRACE, printf("[DO_OOP_WORK:3] reclaim_candidate=%d\n",
                            d->bool_base_value));

        s->sub_stage = 4;
        break;
    }

    case 4:
    {
        if (!d->bool_base_value)
        {
            IFDEF(TRACE, printf("[DO_OOP_WORK:4] not reclaim candidate, "
                                "continue AOP\n"));
            s->sub_stage = 6;
            break;
        }

        d->bool_base_value = false;

        IFDEF(TRACE, printf("[DO_OOP_WORK:4] clear reclaim candidate "
                            "region=%u\n",
                            d->region));

        if (!hwgc_access(s,
                         d->pars.humogousReclaimCandidateBoolBase + d->region,
                         &d->bool_base_value, 1, true))
            return;

        s->sub_stage = 5;
        break;
    }

    case 5:
    {
        uintptr_t region_attr_dest =
            d->pars.regionAttrBase + d->region * 2;
        int8_t dest_value = -1;

        IFDEF(TRACE, printf("[DO_OOP_WORK:5] mark region old, "
                            "attr_addr=%lx value=%d\n",
                            region_attr_dest + 1, dest_value));

        if (!hwgc_access(s, region_attr_dest + 1,
                         &dest_value, 1, true))
            return;

        s->sub_stage = 6;
        break;
    }

    case 6:
    {
        if (d->scanning_in_young)
        {
            IFDEF(TRACE, printf("[DO_OOP_WORK:6] scanning young, "
                                "skip AOP and return previous\n"));

            hwgc_return_previous(s);
        }
        else
        {
            d->aop_region_attr = (uint16_t)d->region_attr_ptr;
            d->aop_dest = d->dest;

            IFDEF(TRACE, printf("[DO_OOP_WORK:6] goto AOP_WORK "
                                "aop_dest=%lx aop_attr=0x%x\n",
                                d->aop_dest, d->aop_region_attr));

            hwgc_goto_stage(s, STAGE_AOP_WORK, 0);
        }
        break;
    }

    case 7:
    {
        IFDEF(TRACE, printf("[DO_OOP_WORK:7] return previous stage\n"));
        hwgc_return_previous(s);
        break;
    }

    default:
        IFDEF(TRACE, printf("[DO_OOP_WORK] invalid sub=%u, reset\n",
                            s->sub_stage));
        hwgc_goto_stage(s, STAGE_DO_OOP_WORK, 0);
        break;
    }
}

static void stage_aop_work_function(HWGCDevState *s)
{
    struct HWGCStageData *d = &s->stageData;

    IFDEF(TRACE, printf("[AOP_WORK] sub=%u aop_dest=%lx aop_attr=0x%x "
                        "card_index=%lx index=%lx buffer=%lx\n",
                        s->sub_stage, d->aop_dest, d->aop_region_attr,
                        d->card_index, d->index, d->buffer));

    switch (s->sub_stage)
    {
    case 0:
    {
        if ((d->aop_region_attr & 0xff) == 0)
        {
            IFDEF(TRACE, printf("[AOP_WORK:0] no remembered-set update needed\n"));
            hwgc_return_previous(s);
            return;
        }

        if (!hwgc_access(s, d->pars.cardTablePtr + 0x38,
                         &d->byte_map, 8, false))
            return;

        if (!hwgc_access(s, d->pars.cardTablePtr + 0x40,
                         &d->byte_map_base, 8, false))
            return;

        d->res = d->byte_map_base + (d->aop_dest >> 9);
        d->card_index = d->res - d->byte_map;

        IFDEF(TRACE, printf("[AOP_WORK:0] byte_map=%lx base=%lx "
                            "card_addr=%lx card_index=%lx\n",
                            d->byte_map, d->byte_map_base,
                            d->res, d->card_index));

        s->sub_stage = 1;
        break;
    }

    case 1:
    {
        if (!hwgc_access(s, d->pars.pss + 0x1b0,
                         &d->last_index, 8, false))
            return;

        IFDEF(TRACE, printf("[AOP_WORK:1] last_index=%lx card_index=%lx\n",
                            d->last_index, d->card_index));

        if (d->card_index == d->last_index)
        {
            IFDEF(TRACE, printf("[AOP_WORK:1] duplicate card, return previous\n"));
            hwgc_return_previous(s);
            return;
        }

        if (!hwgc_access(s, d->pars.pss + 0x48,
                         &d->index, 8, false))
            return;

        if (!hwgc_access(s, d->pars.pss + 0x58,
                         &d->buffer, 8, false))
            return;

        IFDEF(TRACE, printf("[AOP_WORK:1] queue index=%lx buffer=%lx\n",
                            d->index, d->buffer));

        s->sub_stage = 2;
        break;
    }

    case 2:
    {
        if (d->index == 0)
        {
            d->old_node = 0;

            IFDEF(TRACE, printf("[AOP_WORK:2] buffer full/empty, "
                                "prepare node replacement\n"));

            if (!hwgc_access(s, d->pars.pss + 0x20,
                             &d->node_allocator_ptr, 8, false))
                return;
            if (!hwgc_access(s, d->pars.pss + 0x30,
                             &d->offset30, 8, false))
                return;
            if (!hwgc_access(s, d->pars.pss + 0x38,
                             &d->offset38, 8, false))
                return;

            IFDEF(TRACE, printf("[AOP_WORK:2] node_allocator=%lx "
                                "offset30=%lx offset38=%lx\n",
                                d->node_allocator_ptr,
                                d->offset30, d->offset38));

            if (d->buffer != 0)
            {
                uintptr_t zero = 0;

                d->old_node = d->buffer - 0x10;

                IFDEF(TRACE, printf("[AOP_WORK:2] release old node=%lx\n",
                                    d->old_node));

                if (!hwgc_access(s, d->old_node, &zero, 8, true))
                    return;
                if (!hwgc_access(s, d->old_node + 8,
                                 &d->offset30, 8, true))
                    return;
                if (!hwgc_access(s, d->pars.pss + 0x30,
                                 &d->old_node, 8, true))
                    return;

                if (d->offset38 == 0)
                {
                    if (!hwgc_access(s, d->pars.pss + 0x38,
                                     &d->old_node, 8, true))
                        return;
                }
            }

            s->sub_stage = 3;
        }
        else
        {
            IFDEF(TRACE, printf("[AOP_WORK:2] current buffer has room, "
                                "goto append\n"));
            s->sub_stage = 7;
        }
        break;
    }

    case 3:
    {
        d->new_top = 0;

        IFDEF(TRACE, printf("[AOP_WORK:3] pop free node from addr=%lx\n",
                            d->node_allocator_ptr + 0x80));

        if (!hwgc_access(s, d->node_allocator_ptr + 0x80,
                         &d->old_node, 8, false))
            return;

        IFDEF(TRACE, printf("[AOP_WORK:3] free-list head node=%lx\n",
                            d->old_node));

        if (d->old_node != 0)
        {
            if (!hwgc_access(s, d->old_node + 0x8,
                             &d->new_top, 8, false))
                return;

            uintptr_t zero = 0;
            if (!hwgc_access(s, d->old_node + 0x8,
                             &zero, 8, true))
                return;

            IFDEF(TRACE, printf("[AOP_WORK:3] node=%lx next=%lx \n",
                                d->old_node, d->new_top));
        }

        s->sub_stage = 4;
        break;
    }

    case 4:
    {
        IFDEF(TRACE, printf("[AOP_WORK:4] update free-list head "
                            "addr=%lx new_top=%lx\n",
                            d->node_allocator_ptr + 0x80,
                            d->new_top));

        if (!hwgc_access(s, d->node_allocator_ptr + 0x80,
                         &d->new_top, 8, true))
            return;

        s->sub_stage = 5;
        break;
    }

    case 5:
    {
        if (d->old_node == 0)
        {
            IFDEF(TRACE, printf("[AOP_WORK:5] no free node, "
                                "raise IRQ_ALLOCATE\n"));

            s->irq_par0 = d->node_allocator_ptr;
            s->timer_running = false;
            s->irq_to_sub_stage = 6;
            qatomic_or(&s->status, ST_WAIT_ALLOCATE);

            hwgc_raise_irq_from_worker(s, IRQ_ALLOCATE);
            return;
        }

        IFDEF(TRACE, printf("[AOP_WORK:5] acquired node=%lx\n", d->old_node));
        s->sub_stage = 6;
        break;
    }

    case 6:
    {
        d->buffer = d->old_node + 0x10;

        if (!hwgc_access(s, d->pars.pss + 0x58, &d->buffer, 8, true))
            return;

        IFDEF(TRACE, printf("[AOP_WORK:6] new buffer=%lx, "
                            "read node index addr=%lx\n",
                            d->buffer, d->node_allocator_ptr));

        if (!hwgc_access(s, d->node_allocator_ptr,
                         &d->index, 8, false))
            return;

        d->index = d->index * 8;

        IFDEF(TRACE, printf("[AOP_WORK:6] new buffer index=%lx\n",
                            d->index));

        s->sub_stage = 7;
        break;
    }

    case 7:
    {
        int idx = d->index / 8 - 1;

        IFDEF(TRACE, printf("[AOP_WORK:7] append card=%lx "
                            "buffer_slot=%d addr=%lx\n",
                            d->card_index, idx,
                            d->buffer + idx * 8));

        if (!hwgc_access(s, d->buffer + idx * 8,
                         &d->res, 8, true))
            return;

        d->index -= 8;

        if (!hwgc_access(s, d->pars.pss + 0x48,
                         &d->index, 8, true))
            return;

        if (!hwgc_access(s, d->pars.pss + 0x1b0,
                         &d->card_index, 8, true))
            return;

        IFDEF(TRACE, printf("[AOP_WORK:7] appended, "
                            "new_index=%lx last_index=%lx\n",
                            d->index, d->card_index));

        hwgc_return_previous(s);
        break;
    }

    default:
        IFDEF(TRACE, printf("[AOP_WORK] invalid sub=%u, reset\n",
                            s->sub_stage));

        hwgc_goto_stage(s, STAGE_AOP_WORK, 0);
        break;
    }
}

static void do_hwgc_work(void *opaque)
{
    HWGCDevState *s = opaque;

    switch (s->stage)
    {
    case STAGE_FETCH:
        qemu_mutex_lock(&s->lock);
        stage_fetch_function(s);
        hwgc_arm_timer(s);
        qemu_mutex_unlock(&s->lock);
        break;

    case STAGE_PARTIAL_ARRAY:
        qemu_mutex_lock(&s->lock);
        stage_partial_array_function(s);
        hwgc_arm_timer(s);
        qemu_mutex_unlock(&s->lock);
        break;

    case STAGE_COMMON_OOP:
        qemu_mutex_lock(&s->lock);
        stage_oop_function(s);
        hwgc_arm_timer(s);
        qemu_mutex_unlock(&s->lock);
        break;

    case STAGE_COPY2SURVIVOR:
        qemu_mutex_lock(&s->lock);
        stage_copy2survivor_function(s);
        hwgc_arm_timer(s);
        qemu_mutex_unlock(&s->lock);
        break;

    case STAGE_ALLOC:
        qemu_mutex_lock(&s->lock);
        stage_alloc_function(s);
        hwgc_arm_timer(s);
        qemu_mutex_unlock(&s->lock);
        break;

    case STAGE_ALLOCATE_DIRECT:
        qemu_mutex_lock(&s->lock);
        stage_allocate_direct_function(s);
        hwgc_arm_timer(s);
        qemu_mutex_unlock(&s->lock);
        break;

    case STAGE_ALLOCATE_DURING_GC:
        qemu_mutex_lock(&s->lock);
        stage_allocate_during_gc_function(s);
        hwgc_arm_timer(s);
        qemu_mutex_unlock(&s->lock);
        break;

    case STAGE_PAR_ALLOCATE_IML:
        qemu_mutex_lock(&s->lock);
        stage_par_allocate_iml_function(s);
        hwgc_arm_timer(s);
        qemu_mutex_unlock(&s->lock);
        break;

    case STAGE_PAR_ALLOCATE:
        qemu_mutex_lock(&s->lock);
        stage_par_allocate_function(s);
        hwgc_arm_timer(s);
        qemu_mutex_unlock(&s->lock);
        break;

        //    case STAGE_ATTEMPT_ALLOC:
        //        stage_attempt_alloc_function(s);
        //        break;
        //
        //    case STAGE_NEW_GC_ALLOC:
        //        stage_new_gc_alloc_function(s);
        //        break;
        //
        //    case STAGE_ALLOC_FREE_REGION:
        //        stage_alloc_free_region_function(s);
        //        break;

    case STAGE_COPY:
        qemu_mutex_lock(&s->lock);
        stage_copy_function(s);
        hwgc_arm_timer(s);
        qemu_mutex_unlock(&s->lock);
        break;

    case STAGE_TRACE:
        qemu_mutex_lock(&s->lock);
        stage_trace_function(s);
        hwgc_arm_timer(s);
        qemu_mutex_unlock(&s->lock);
        break;

    case STAGE_TRACE_PLUS:
        qemu_mutex_lock(&s->lock);
        stage_trace_plus_function(s);
        hwgc_arm_timer(s);
        qemu_mutex_unlock(&s->lock);
        break;

    case STAGE_TRACE_DEC:
        qemu_mutex_lock(&s->lock);
        stage_trace_dec_function(s);
        hwgc_arm_timer(s);
        qemu_mutex_unlock(&s->lock);
        break;

    case STAGE_DO_OOP_WORK:
        qemu_mutex_lock(&s->lock);
        stage_do_oop_work_function(s);
        hwgc_arm_timer(s);
        qemu_mutex_unlock(&s->lock);
        break;

    case STAGE_AOP_WORK:
        qemu_mutex_lock(&s->lock);
        stage_aop_work_function(s);
        hwgc_arm_timer(s);
        qemu_mutex_unlock(&s->lock);
        break;

    case STAGE_DONE:
        qemu_mutex_lock(&s->lock);
        s->timer_running = false;
        qatomic_and(&s->status, ~ST_BUSY);
        qatomic_or(&s->status, ST_DONE);

        hwgc_raise_irq_from_worker(s, IRQ_DONE);
        qemu_mutex_unlock(&s->lock);
        break;

    default:
        break;
    }
}

static void hwgc_tick_timer_cb(void *opaque)
{
    HWGCDevState *s = opaque;

    qemu_mutex_lock(&s->lock);

    // tick_pending表示有一次的tick需要处理
    if (!s->thread_stop && s->timer_running)
    {
        s->tick_pending = true;
        qemu_cond_signal(&s->cond);
    }

    qemu_mutex_unlock(&s->lock);
}

static void *hwgc_worker_thread(void *opaque)
{
    HWGCDevState *s = opaque;

    while (true)
    {
        qemu_mutex_lock(&s->lock);

        while (!s->thread_stop && !s->tick_pending)
            qemu_cond_wait(&s->cond, &s->lock);

        if (s->thread_stop)
        {
            qemu_mutex_unlock(&s->lock);
            break;
        }

        // 消费掉这一次的tick 执行一个设备周期
        s->tick_pending = false;
        qemu_mutex_unlock(&s->lock);

        do_hwgc_work(s);
    }

    return NULL;
}

static void hwgc_reset_device_locked(HWGCDevState *s)
{
    s->timer_running = false;
    s->tick_pending = false;

    qatomic_set(&s->status, qatomic_read(&s->status) & ST_IRQ_EN);
    qatomic_set(&s->irq_status, 0);

    s->stage = STAGE_IDLE;
    s->sub_stage = 0;

    memset(&s->stageData, 0, sizeof(struct HWGCStageData));
    s->irq_par0 = 0;
    s->irq_par1 = 0;
    s->irq_res0 = 0;
    s->irq_res1 = 0;

    hwgc_tlb_flush(s);
}

static void hwgc_start_locked(HWGCDevState *s)
{
    if (qatomic_read(&s->status) & ST_BUSY)
        return;

    qatomic_and(&s->status, ST_IRQ_EN);
    qatomic_or(&s->status, ST_BUSY);

    qatomic_set(&s->irq_status, 0);

    s->stage = STAGE_FETCH;
    s->sub_stage = 0;

    s->stageData.localBot = s->stageData.pars.localBot;

    hwgc_tlb_flush(s);

    s->timer_running = true;
    s->tick_pending = false;
    hwgc_arm_timer(s);
}

static uint64_t hwgc_mmio_read(void *opaque, hwaddr addr, unsigned size)
{
    HWGCDevState *s = opaque;
    uint64_t ret = ~0ULL;

    qemu_mutex_lock(&s->lock);

    switch (addr)
    {
    case REG_STATUS:
        ret = qatomic_read(&s->status);
        break;
    case REG_IRQ_STATUS:
        ret = qatomic_read(&s->irq_status);
        break;
    case REG_IRQ_PAR0:
        ret = s->irq_par0;
        break;
    case REG_IRQ_PAR1:
        ret = s->irq_par1;
        break;
    default:
        IFDEF(TRACE, printf("[hwgc] unknown MMIO read: addr=0x%" HWADDR_PRIx
                            ", size=%u\n",
                            addr, size));
        ret = ~0ULL;
        break;
    }

    IFDEF(TRACE, printf("[hwgc] MMIO read: addr=0x%" HWADDR_PRIx
                        ", size=%u, value=0x%016" PRIx64 "\n",
                        addr, size, ret));

    qemu_mutex_unlock(&s->lock);
    return ret;
}

static void hwgc_mmio_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    HWGCDevState *s = opaque;
    bool do_continue = false;

    IFDEF(TRACE, printf("[hwgc] MMIO write: addr=0x%" HWADDR_PRIx
                        ", size=%u, value=0x%016" PRIx64 "\n",
                        addr, size, val));

    if (addr == REG_IRQ_CLEAR && size == 4)
    {
        IFDEF(TRACE, printf("[hwgc] IRQ clear: value=0x%08" PRIx64 "\n",
                            val & UINT64_C(0xffffffff)));

        hwgc_lower_irq_from_mmio(s, val);
        return;
    }

    qemu_mutex_lock(&s->lock);

    switch (addr)
    {
    case REG_STATUS:
        if (size == 4)
        {
            if (val & ST_IRQ_EN)
            {
                IFDEF(TRACE, printf("[hwgc] enable IRQ\n"));
                qatomic_or(&s->status, ST_IRQ_EN);
            }
            else
            {
                IFDEF(TRACE, printf("[hwgc] disable IRQ\n"));
                qatomic_and(&s->status, ~ST_IRQ_EN);
            }
        }
        else
            IFDEF(TRACE, printf("[hwgc] invalid STATUS size: %u\n", size));
        break;

    case REG_CMD:
        if (size == 4)
        {
            if (val & CMD_RESET)
            {
                IFDEF(TRACE, printf("[hwgc] CMD_RESET\n"));
                hwgc_reset_device_locked(s);
            }

            if (val & CMD_START)
                hwgc_start_locked(s);

            if (val & CMD_CONTINUE)
            {
                if (s->status & ST_WAIT_TLB)
                {
                    hwgc_tlb_insert_locked(s, s->irq_res0, s->irq_res1);
                    qatomic_and(&s->status, ~ST_WAIT_TLB);
                }
                else if (s->status & ST_WAIT_ALLOCATE)
                {
                    s->stageData.old_node = s->irq_res0;
                    s->sub_stage = s->irq_to_sub_stage;
                    qatomic_and(&s->status, ~ST_WAIT_ALLOCATE);
                }
                else if (s->status & ST_WAIT_WAKE)
                {
                    s->sub_stage = s->irq_to_sub_stage;
                    qatomic_and(&s->status, ~ST_WAIT_WAKE);
                }

                s->timer_running = true;
                s->tick_pending = false;
                do_continue = true;
            }
        }
        else
            IFDEF(TRACE, printf("[hwgc] invalid CMD size: %u\n", size));
        break;

    case REG_IRQ_RES0:
        if (size == 8)
            s->irq_res0 = val;
        break;

    case REG_IRQ_RES1:
        if (size == 8)
            s->irq_res1 = val;
        break;
    }

    if (addr >= REG_PAR0 && addr <= REG_PAR21)
    {
        static const size_t par_offsets[] = {
            offsetof(struct HWGCParameters, chunkSize),                        // REG_PAR0: lo=chunkSize, hi=ageThreshold
            offsetof(struct HWGCParameters, heapRegionBias),                   // REG_PAR1: lo=heapRegionBias, hi=heapRegionShiftBy
            offsetof(struct HWGCParameters, regionAttrShiftBy),                // REG_PAR2: lo=regionAttrShiftBy, hi=logOfHRGrainBytes
            offsetof(struct HWGCParameters, localBot),                         // REG_PAR3: lo=locabot, hi=cimpressedFlags
            offsetof(struct HWGCParameters, stepperOffset),                    // REG_PAR4
            offsetof(struct HWGCParameters, youngWordsBase),                   // REG_PAR5
            offsetof(struct HWGCParameters, regionAttrBase),                   // REG_PAR6
            offsetof(struct HWGCParameters, plabAllocatorPtr),                 // REG_PAR7
            offsetof(struct HWGCParameters, regionAttrBiasedBase),             // REG_PAR8
            offsetof(struct HWGCParameters, heapRegionBiasedBase),             // REG_PAR0
            offsetof(struct HWGCParameters, pss),                              // REG_PAR10
            offsetof(struct HWGCParameters, taskQueueElemsBase),               // REG_PAR11
            offsetof(struct HWGCParameters, humogousReclaimCandidateBoolBase), // REG_PAR12
            offsetof(struct HWGCParameters, cardTablePtr),                     // REG_PAR13
            offsetof(struct HWGCParameters, g1h),                              // REG_PAR14
            offsetof(struct HWGCParameters, intArrayKlassObj),                 // REG_PAR15
            offsetof(struct HWGCParameters, objectKlass),                      // REG_PAR16
            offsetof(struct HWGCParameters, lockPtr),                          // REG_PAR17
            offsetof(struct HWGCParameters, thread),                           // REG_PAR18
            offsetof(struct HWGCParameters, dummyRegion),                      // REG_PAR19
            offsetof(struct HWGCParameters, compressedOopBase),                // REG_PAR20
            offsetof(struct HWGCParameters, compressedKlassPointerBase),       // REG_PAR21

        };
        int idx = (addr - REG_PAR0) / 8;
        uint8_t *base = (uint8_t *)&s->stageData.pars;
        if (idx <= 2)
        {
            uint32_t *lo = (uint32_t *)(base + par_offsets[idx]);
            uint32_t *hi = lo + 1;
            *lo = (uint32_t)val;
            *hi = (uint32_t)(val >> 32);
        }
        else if (idx == 3)
        {
            s->stageData.pars.localBot = (uint32_t)val;
            s->stageData.pars.useCompressedOops = (uint8_t)(val >> 32);
            s->stageData.pars.compressedOopShift = (uint8_t)(val >> 40);
            s->stageData.pars.useCompressedKlassPointers = (uint8_t)(val >> 48);
            s->stageData.pars.compressedKlassPointerShift = (uint8_t)(val >> 56);
        }
        else
            *(uint64_t *)(base + par_offsets[idx]) = val;
    }

    qemu_mutex_unlock(&s->lock);

    if (do_continue)
    {
        IFDEF(TRACE, printf("[hwgc] arm timer, period=%" PRIu64 " ns\n",
                            s->period_ns));
        hwgc_arm_timer(s);
    }
}

static const MemoryRegionOps hwgc_mmio_ops = {
    .read = hwgc_mmio_read,
    .write = hwgc_mmio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 8,
    },
    .impl = {
        .min_access_size = 4,
        .max_access_size = 8,
    },
};

static void hwgc_realize(PCIDevice *pdev, Error **errp)
{
    HWGCDevState *s = HWGC_DEV(pdev);
    uint8_t *pci_conf = pdev->config;

    pci_config_set_interrupt_pin(pci_conf, 1);

    if (msi_init(pdev, 0, 1, true, false, errp))
        return;

    qemu_mutex_init(&s->lock);
    qemu_cond_init(&s->cond);

    s->period_ns = 1; /* default: 1 ms per simulated hardware cycle */
    s->thread_stop = false;
    s->timer_running = false;
    s->tick_pending = false;
    s->stage = STAGE_IDLE;
    hwgc_tlb_flush(s);

    s->tick_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, hwgc_tick_timer_cb, s);

    qemu_thread_create(&s->worker, "hwgc", hwgc_worker_thread, s, QEMU_THREAD_JOINABLE);
    memory_region_init_io(&s->mmio, OBJECT(s), &hwgc_mmio_ops, s, "hwgc-mmio", 4 * KiB);
    pci_register_bar(pdev, 0, PCI_BASE_ADDRESS_SPACE_MEMORY, &s->mmio);
}

static void hwgc_exit(PCIDevice *pdev)
{
    HWGCDevState *s = HWGC_DEV(pdev);

    qemu_mutex_lock(&s->lock);
    s->thread_stop = true;
    s->timer_running = false;
    s->tick_pending = true;
    qemu_cond_signal(&s->cond);
    qemu_mutex_unlock(&s->lock);

    if (s->tick_timer)
    {
        timer_del(s->tick_timer);
        timer_free(s->tick_timer);
        s->tick_timer = NULL;
    }

    qemu_thread_join(&s->worker);

    qemu_cond_destroy(&s->cond);
    qemu_mutex_destroy(&s->lock);

    msi_uninit(pdev);
}

static void hwgc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

    k->realize = hwgc_realize;
    k->exit = hwgc_exit;
    k->vendor_id = HWGC_VENDOR_ID;
    k->device_id = HWGC_DEVICE_ID;
    k->revision = 0x01;
    k->class_id = PCI_CLASS_OTHERS;

    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

static const TypeInfo hwgc_info = {
    .name = TYPE_HWGC_DEV,
    .parent = TYPE_PCI_DEVICE,
    .instance_size = sizeof(HWGCDevState),
    .class_init = hwgc_class_init,
    .interfaces = (const InterfaceInfo[]){
        {INTERFACE_CONVENTIONAL_PCI_DEVICE},
        {},
    },
};

static void hwgc_register_types(void)
{
    type_register_static(&hwgc_info);
}

type_init(hwgc_register_types)