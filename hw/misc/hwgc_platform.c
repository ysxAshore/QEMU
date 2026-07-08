#include "hwgc_platform.h"

#include "qemu/units.h"
#include "qemu/timer.h"
#include "qemu/main-loop.h"
#include "qemu/module.h"
#include "qom/object.h"
#include "system/address-spaces.h"
#include "hw/sysbus.h"

#define TRACE 1
#define IRQDEBUG 1
#define IFDEF(cond, stmt)           \
    if (cond)                       \
        do                          \
        {                           \
            printf("[HWGC %p]", s); \
            stmt;                   \
        } while (0);

#define TYPE_HWGC_PLATFORM_DEV "hwgc-platform"
typedef struct HWGCPlatformDevState HWGCPlatformDevState;

static void stage_fetch_function(HWGCPlatformDevState *s);
static void stage_partial_array_function(HWGCPlatformDevState *s);
static void stage_oop_function(HWGCPlatformDevState *s);
static void stage_copy2survivor_function(HWGCPlatformDevState *s);
static void stage_alloc_function(HWGCPlatformDevState *s);
static void stage_allocate_direct_function(HWGCPlatformDevState *s);
static void stage_allocate_during_gc_function(HWGCPlatformDevState *s);
static void stage_attempt_alloc_function(HWGCPlatformDevState *s);
static void stage_new_gc_alloc_function(HWGCPlatformDevState *s);
static void stage_alloc_free_region_function(HWGCPlatformDevState *s);
static void stage_par_allocate_iml_function(HWGCPlatformDevState *s);
static void stage_par_allocate_function(HWGCPlatformDevState *s);
static void stage_copy_function(HWGCPlatformDevState *s);
static void stage_trace_function(HWGCPlatformDevState *s);
static void stage_trace_plus_function(HWGCPlatformDevState *s);
static void stage_trace_dec_function(HWGCPlatformDevState *s);
static void stage_do_oop_work_function(HWGCPlatformDevState *s);
static void stage_aop_work_function(HWGCPlatformDevState *s);

static uint32_t hwgc_tlb_hash(uint64_t va_page)
{
    return (va_page >> HWGC_PAGE_SHIFT) & (HWGC_TLB_SIZE - 1);
}

static void hwgc_tlb_flush(HWGCPlatformDevState *s)
{
    memset(s->tlb, 0, sizeof(s->tlb));
}

static void hwgc_tlb_insert_locked(HWGCPlatformDevState *s, uint64_t va, hwaddr pa)
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

static bool hwgc_tlb_lookup_locked(HWGCPlatformDevState *s, uint64_t va, hwaddr *pa)
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

static void hwgc_raise_irq_from_worker(HWGCPlatformDevState *s, uint32_t bits)
{
    qatomic_or(&s->irq_status, bits);

    if (!(qatomic_read(&s->status) & ST_IRQ_EN))
        return;

    qemu_set_irq(s->irq, 1);
}

static void hwgc_lower_irq_from_mmio(HWGCPlatformDevState *s, uint32_t bits)
{
    qatomic_and(&s->irq_status, ~bits);

    if (!qatomic_read(&s->irq_status))
        qemu_set_irq(s->irq, 0);
}

static void hwgc_arm_timer(HWGCPlatformDevState *s)
{
    timer_mod_ns(s->tick_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + s->period_ns);
}

static void hwgc_pause_for_tlb_miss(HWGCPlatformDevState *s, uint64_t va, uint32_t access)
{
    s->irq_par0 = va;
    s->irq_par1 = access;
    s->timer_running = false;
    qatomic_or(&s->status, ST_WAIT_TLB);

    hwgc_raise_irq_from_worker(s, IRQ_TLB_MISS);
}

static bool hwgc_translate(HWGCPlatformDevState *s, uint64_t va, uint32_t access, hwaddr *pa)
{
    bool hit;

    hit = hwgc_tlb_lookup_locked(s, va, pa);

    if (!hit)
    {
        IFDEF(TRACE, printf("translate not hit\n"));
        hwgc_pause_for_tlb_miss(s, va, access);
        return false;
    }

    return true;
}

static bool hwgc_access(HWGCPlatformDevState *s, uint64_t va, void *val, uint size, bool write)
{
    hwaddr pa;
    MemTxResult tx;
    int align = size - 1;

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

static bool hwgc_cmpxchg(HWGCPlatformDevState *s, uintptr_t vaddr, uint64_t old_val, uint64_t new_val, int size, void *return_value)
{
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

static inline void hwgc_goto_stage(HWGCPlatformDevState *s, int stage, int sub_stage)
{
    s->stage = stage;
    s->sub_stage = sub_stage;
}

static inline void hwgc_return_previous(HWGCPlatformDevState *s)
{
    s->stage = s->stageData.previous;
    s->sub_stage = s->stageData.previous_sub_stage;
}

static inline void hwgc_return_done(HWGCPlatformDevState *s)
{
    s->stage = s->stageData.done_to;
    s->sub_stage = s->stageData.doneto_sub_stage;
}

static void hwgc_platform_stage_dispatch(HWGCPlatformDevState *s)
{
    switch (s->stage)
    {
    case STAGE_FETCH:
        stage_fetch_function(s);
        break;
    case STAGE_PARTIAL_ARRAY:
        stage_partial_array_function(s);
        break;
    case STAGE_COMMON_OOP:
        stage_oop_function(s);
        break;
    case STAGE_COPY2SURVIVOR:
        stage_copy2survivor_function(s);
        break;
    case STAGE_ALLOC:
        stage_alloc_function(s);
        break;
    case STAGE_ALLOCATE_DIRECT:
        stage_allocate_direct_function(s);
        break;
    case STAGE_ALLOCATE_DURING_GC:
        stage_allocate_during_gc_function(s);
        break;
    case STAGE_PAR_ALLOCATE_IML:
        stage_par_allocate_iml_function(s);
        break;
    case STAGE_PAR_ALLOCATE:
        stage_par_allocate_function(s);
        break;
    case STAGE_ATTEMPT_ALLOC:
        stage_attempt_alloc_function(s);
        break;
    case STAGE_NEW_GC_ALLOC:
        stage_new_gc_alloc_function(s);
        break;
    case STAGE_ALLOCATE_FREE:
        stage_alloc_free_region_function(s);
        break;
    case STAGE_COPY:
        stage_copy_function(s);
        break;
    case STAGE_TRACE:
        stage_trace_function(s);
        break;
    case STAGE_TRACE_PLUS:
        stage_trace_plus_function(s);
        break;
    case STAGE_TRACE_DEC:
        stage_trace_dec_function(s);
        break;
    case STAGE_DO_OOP_WORK:
        stage_do_oop_work_function(s);
        break;
    case STAGE_AOP_WORK:
        stage_aop_work_function(s);
        break;
    case STAGE_DONE:
        s->timer_running = false;
        qatomic_and(&s->status, ~ST_BUSY);
        qatomic_or(&s->status, ST_DONE);
        hwgc_raise_irq_from_worker(s, IRQ_DONE);
        break;
    default:
        break;
    }
}

static void stage_fetch_function(HWGCPlatformDevState *s)
{
    assert(s->sub_stage == 0);
    struct HWGCStageData *d = &s->stageData;

    if (d->localBot == 0)
    {
        hwgc_goto_stage(s, STAGE_DONE, 0);
        return;
    }

    uint elems_bias = (d->localBot - 1) & ((1 << 17) - 1);
    IFDEF(TRACE, printf("elems bias %x, addr %lx\n", elems_bias, d->pars.taskQueueElemsBase + elems_bias * 8));
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
    IFDEF(TRACE, printf("dispatch task %lx\n", d->task));
}

static void stage_partial_array_function(HWGCPlatformDevState *s)
{
    struct HWGCStageData *d = &s->stageData;

    IFDEF(TRACE, printf("[PARTIAL_ARRAY] sub=%u task=%lx from=%lx to=%lx localBot=%u\n",
                        s->sub_stage, d->task, d->from_obj, d->to_obj, d->localBot));

    switch (s->sub_stage)
    {
    case 0:
    {
        d->from_obj = d->task;
        if (!hwgc_access(s, d->from_obj, &d->partial_m_value, 8, false))
            return;
        d->to_obj = d->partial_m_value & ~0x3;
        uintptr_t from_len_addr = d->from_obj + (d->pars.useCompressedKlassPointers ? 12 : 16);
        if (!hwgc_access(s, from_len_addr, &d->partial_from_length, 4, false))
            return;
        s->sub_stage = 1;
        break;
    }
    case 1:
    {
        int temp;
        uintptr_t to_len_addr = d->to_obj + (d->pars.useCompressedKlassPointers ? 12 : 16);
        if (!hwgc_access(s, to_len_addr, &d->start, 4, false))
            return;
        temp = d->start + d->pars.chunkSize;
        if (!hwgc_access(s, to_len_addr, &temp, 4, true))
            return;
        s->sub_stage = 2;
        break;
    }
    case 2:
    {
        uint32_t task_num = d->start / d->pars.chunkSize;
        uint32_t remaining_tasks = (d->partial_from_length - d->start) / d->pars.chunkSize;
        uint32_t task_limit = (uint32_t)d->pars.stepperOffset;
        uint32_t task_fanout = d->pars.stepperOffset >> 32;
        uint32_t max_pending = (task_fanout - 1) * task_num + 1;
        uint32_t pending = MIN(max_pending, MIN(remaining_tasks, task_limit));
        d->ncreate = MIN(task_fanout, MIN(remaining_tasks, task_limit + 1) - pending);
        d->i = 0;
        s->sub_stage = 3;
        break;
    }
    case 3:
        if (d->i >= d->ncreate)
        {
            s->sub_stage = 4;
            break;
        }
        {
            uintptr_t pushData = d->from_obj + 0x2;
            uintptr_t queue_addr = d->pars.taskQueueElemsBase + d->localBot * 8;
            if (!hwgc_access(s, queue_addr, &pushData, 8, true))
                return;
            d->localBot = (d->localBot + 1) & ((1 << 17) - 1);
            d->i++;
        }
        break;
    case 4:
    {
        uintptr_t heap_region_ptr = d->pars.heapRegionBiasedBase + (d->to_obj >> d->pars.heapRegionShiftBy) * 8;
        if (!hwgc_access(s, heap_region_ptr, &d->heap_region, 8, false))
            return;
        s->sub_stage = 5;
        break;
    }
    case 5:
        if (!hwgc_access(s, d->heap_region + 0xbc, &d->heap_region_type, 4, false))
            return;
        s->sub_stage = 6;
        break;
    case 6:
    {
        d->scanning_in_young = (d->heap_region_type & 0x2) != 0;
        size_t oop_size = d->pars.useCompressedOops ? 4 : 8;
        uintptr_t base = d->to_obj + (d->pars.useCompressedKlassPointers ? 16 : 24);
        uintptr_t low = base + d->start * oop_size;
        uintptr_t high = base + (d->start + d->pars.chunkSize) * oop_size;
        d->p = base;
        d->q = base + (d->start + d->pars.chunkSize) * oop_size;
        if (d->p < low)
            d->p = low;
        if (d->q > high)
            d->q = high;
        d->previous = STAGE_TRACE_PLUS;
        d->previous_sub_stage = 0;
        d->done_to = STAGE_FETCH;
        d->doneto_sub_stage = 0;
        hwgc_goto_stage(s, STAGE_TRACE_PLUS, 0);
        break;
    }
    default:
        hwgc_goto_stage(s, STAGE_PARTIAL_ARRAY, 0);
        break;
    }
}

static void stage_oop_function(HWGCPlatformDevState *s)
{
    struct HWGCStageData *d = &s->stageData;
    switch (s->sub_stage)
    {
    case 0:
    {
        uint size = d->pars.useCompressedOops ? 4 : 8;
        d->offset = 0;
        if (!hwgc_access(s, d->task, &d->offset, size, false))
            return;
        s->sub_stage = 1;
        break;
    }
    case 1:
        if (d->pars.useCompressedOops)
        {
            uint32_t narrow_oop = (uint32_t)d->offset;
            d->from_obj = (uintptr_t)d->pars.compressedOopBase + ((uintptr_t)narrow_oop << d->pars.compressedOopShift);
        }
        else
        {
            d->from_obj = d->offset;
        }
        if (!hwgc_access(s, d->from_obj, &d->common_m_value, 8, false))
            return;
        s->sub_stage = 2;
        break;
    case 2:
        d->region_attr_ptr = d->pars.regionAttrBiasedBase + (d->from_obj >> d->pars.regionAttrShiftBy) * 2;
        if (!hwgc_access(s, d->region_attr_ptr, &d->src_region_attr, 2, false))
            return;
        if ((int8_t)(d->src_region_attr >> 8) < 0)
        {
            hwgc_goto_stage(s, STAGE_FETCH, 0);
            return;
        }
        s->sub_stage = 3;
        break;
    case 3:
        if ((d->common_m_value & 0x3) == 0x3)
        {
            d->to_obj = d->common_m_value & ~0x3;
            s->sub_stage = 4;
        }
        else
        {
            d->src_region_attr_ptr = d->region_attr_ptr;
            hwgc_goto_stage(s, STAGE_COPY2SURVIVOR, 0);
        }
        break;
    case 4:
    {
        uintptr_t writeObj = d->to_obj;
        if (d->pars.useCompressedOops)
        {
            uint32_t narrow_writeObj = (d->to_obj - d->pars.compressedOopBase) >> d->pars.compressedOopShift;
            if (!hwgc_access(s, d->task, &narrow_writeObj, 4, true))
                return;
        }
        else
        {
            if (!hwgc_access(s, d->task, &writeObj, 8, true))
                return;
        }
        s->sub_stage = 5;
        break;
    }
    case 5:
        if (((d->task ^ d->to_obj) >> d->pars.logOfHRGrainBytes) == 0)
        {
            hwgc_goto_stage(s, STAGE_FETCH, 0);
            return;
        }
        d->region_attr_ptr = 0;
        if (!hwgc_access(s, d->pars.heapRegionBiasedBase + (d->task >> d->pars.heapRegionShiftBy) * 8, &d->heap_region, 8, false))
            return;
        s->sub_stage = 6;
        break;
    case 6:
        if (!hwgc_access(s, d->heap_region + 0xbc, &d->heap_region_type, 4, false))
            return;
        if ((d->heap_region_type & 0x2) != 0)
        {
            hwgc_goto_stage(s, STAGE_FETCH, 0);
            return;
        }
        d->region_attr_ptr = d->pars.regionAttrBiasedBase + (d->to_obj >> d->pars.regionAttrShiftBy) * 2;
        if (!hwgc_access(s, d->region_attr_ptr, &d->aop_region_attr, 2, false))
            return;
        d->aop_dest = d->task;
        d->previous = STAGE_FETCH;
        d->previous_sub_stage = 0;
        hwgc_goto_stage(s, STAGE_AOP_WORK, 0);
        break;
    default:
        hwgc_goto_stage(s, STAGE_COMMON_OOP, 0);
        break;
    }
}

static void stage_copy2survivor_function(HWGCPlatformDevState *s)
{
    struct HWGCStageData *d = &s->stageData;
    int8_t src_region_attr_type = (int8_t)(d->src_region_attr >> 8);
    switch (s->sub_stage)
    {
    case 0:
        if (!hwgc_access(s, d->from_obj + 8, &d->klass_ptr, 8, false))
            return;
        s->sub_stage = 1;
        break;
    case 1:
        if (d->pars.useCompressedKlassPointers)
            d->klass_ptr = d->pars.compressedKlassPointerBase + ((uintptr_t)((uint32_t)d->klass_ptr) << d->pars.compressedKlassPointerShift);
        if (!hwgc_access(s, d->klass_ptr + 8, &d->region_attr_ptr, 8, false))
            return;
        s->sub_stage = 2;
        break;
    case 2:
        d->lh = (int)d->region_attr_ptr;
        d->kid = (int)(d->region_attr_ptr >> 32);
        if (d->lh > 0 && (d->lh & 0x1) == 0)
            d->size = (size_t)d->lh >> 3;
        else if (d->lh < 0)
        {
            uintptr_t len_addr = d->from_obj + (d->pars.useCompressedKlassPointers ? 12 : 16);
            if (!hwgc_access(s, len_addr, &d->common_oop_array_length, 4, false))
                return;
            size_t temp = ((size_t)d->common_oop_array_length << (uint8_t)d->lh) + (uint8_t)(d->lh >> 16);
            d->size = (temp & 0x7) ? ((temp >> 3) + 1) : (temp >> 3);
        }
        else if (d->kid == 2)
        {
            uint oop_size_offset = d->pars.useCompressedKlassPointers ? 0x20 : 0x24;
            d->size = 0;
            if (!hwgc_access(s, d->from_obj + oop_size_offset, &d->size, 4, false))
                return;
        }
        else
            d->size = (size_t)d->lh >> 3;
        s->sub_stage = 3;
        break;
    case 3:
        if (!hwgc_access(s, d->pars.pss + 0x178, &d->dest_attr_cache, 4, false))
            return;
        d->dest_attr = src_region_attr_type == 1 ? d->dest_attr_cache >> 16 : d->dest_attr_cache & 0xffff;
        d->dest_attr_ptr = src_region_attr_type == 1 ? d->pars.pss + 0x178 + 0x2 : d->pars.pss + 0x178;
        s->sub_stage = 4;
        break;
    case 4:
        if (src_region_attr_type == 0)
        {
            if ((d->common_m_value & 0x1) == 0x0)
            {
                uintptr_t ptr = (d->common_m_value & 0x2) ? (d->common_m_value ^ 0x2) : d->common_m_value;
                if (!hwgc_access(s, ptr, &d->monitor_markWord, 8, false))
                    return;
                d->age = (d->monitor_markWord >> 3) & 0x1111;
            }
            else
            {
                d->age = (d->common_m_value >> 3) & 0x1111;
            }
            if (d->age < d->pars.ageThreshold)
            {
                d->dest_attr = d->src_region_attr;
                d->dest_attr_ptr = d->src_region_attr_ptr;
            }
        }
        s->sub_stage = 5;
        break;
    case 5:
    {
        int8_t dest_attr_type = (int8_t)(d->dest_attr >> 8);
        uintptr_t allocator_addr = d->pars.plabAllocatorPtr + 0x10 + dest_attr_type * 8;
        if (!hwgc_access(s, allocator_addr, &d->buffer_temp, 8, false))
            return;
        s->sub_stage = 6;
        break;
    }
    case 6:
        if (!hwgc_access(s, d->buffer_temp, &d->buffer, 8, false))
            return;
        if (!hwgc_access(s, d->buffer + 0x30, &d->region_top, 8, false))
            return;
        if (!hwgc_access(s, d->buffer + 0x38, &d->region_end, 8, false))
            return;
        s->sub_stage = 7;
        break;
    case 7:
        if ((d->region_end - d->region_top) / 8 >= d->size)
        {
            d->to_obj = d->region_top;
            uintptr_t writeValue = d->to_obj + d->size * 8;
            if (!hwgc_access(s, d->buffer + 0x30, &writeValue, 8, true))
                return;
            d->region_top = writeValue;
            s->sub_stage = 8;
        }
        else
        {
            d->to_obj = 0;
            hwgc_goto_stage(s, STAGE_ALLOC, 0);
        }
        break;
    case 8:
    {
        uintptr_t updatedMW = (d->to_obj & ~0x3) | 0x3;
        uintptr_t return_value;
        if (!hwgc_cmpxchg(s, d->from_obj, d->common_m_value, updatedMW, 8, &return_value))
            return;
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
        if ((int8_t)(d->dest_attr >> 8) == 0 && (d->common_m_value & 0x1) != 0)
            new_mark = (d->common_m_value & ~(0x1111 << 3)) | ((((d->age + 1) < 15 ? d->age + 1 : d->age) & 0x1111) << 3);
        if (!hwgc_access(s, d->to_obj, &new_mark, 8, true))
            return;
        s->sub_stage = 10;
        break;
    }
    case 10:
        if ((int8_t)(d->dest_attr >> 8) == 0 && (d->common_m_value & 0x1) == 0)
        {
            uintptr_t ptr = (d->common_m_value & 0x2) ? (d->common_m_value ^ 0x2) : d->common_m_value;
            if (!hwgc_access(s, ptr, &d->region_attr_ptr, 8, false))
                return;
            d->region_attr_ptr = (d->region_attr_ptr & ~(0x1111 << 3)) | ((((d->age + 1) < 15 ? d->age + 1 : d->age) & 0x1111) << 3);
            if (!hwgc_access(s, ptr, &d->region_attr_ptr, 8, true))
                return;
        }
        s->sub_stage = 11;
        break;
    case 11:
        d->i = 1;
        hwgc_goto_stage(s, STAGE_COPY, 0);
        break;
    case 12:
        if (d->kid == 4)
            hwgc_goto_stage(s, STAGE_COMMON_OOP, 4);
        else
            hwgc_goto_stage(s, STAGE_TRACE, 0);
        break;
    case 13:
        if (!hwgc_access(s, d->buffer + 0x28, &d->region_bottom, 8, false))
            return;
        if (!hwgc_access(s, d->buffer + 0x40, &d->region_hard_end, 8, false))
            return;
        if (d->to_obj >= d->region_bottom && d->to_obj < d->region_hard_end)
        {
            if (!hwgc_access(s, d->buffer + 0x30, &d->to_obj, 8, true))
                return;
            d->to_obj = d->forward_ptr;
            hwgc_goto_stage(s, STAGE_COMMON_OOP, 4);
        }
        else
        {
            s->sub_stage = 14;
        }
        break;
    case 14:
    {
        uint words = d->size / 8;
        uintptr_t cur_klass = 0;
        uint header_words = d->pars.useCompressedKlassPointers ? 2 : 3;
        if (words >= header_words)
        {
            uint payload_size = words - header_words;
            uint32_t len = payload_size * 2;
            uintptr_t array_len_addr = d->to_obj + (d->pars.useCompressedKlassPointers ? 12 : 16);
            if (!hwgc_access(s, array_len_addr, &len, 4, true))
                return;
            cur_klass = d->pars.intArrayKlassObj;
        }
        else if (words > 0)
        {
            cur_klass = d->pars.objectKlass;
        }
        uintptr_t mark = 0x1;
        if (!hwgc_access(s, d->to_obj, &mark, 8, true))
            return;
        if (d->pars.useCompressedKlassPointers)
        {
            uint32_t narrow_klass = (uint32_t)((cur_klass - s->stageData.pars.compressedKlassPointerBase) >> s->stageData.pars.compressedKlassPointerShift);
            if (!hwgc_access(s, d->to_obj + 0x8, &narrow_klass, 4, true))
                return;
        }
        else
        {
            if (!hwgc_access(s, d->to_obj + 0x8, &cur_klass, 8, true))
                return;
        }
        d->to_obj = d->forward_ptr;
        hwgc_goto_stage(s, STAGE_COMMON_OOP, 4);
        break;
    }
    default:
        hwgc_goto_stage(s, STAGE_COPY2SURVIVOR, 0);
        break;
    }
}

static void stage_alloc_function(HWGCPlatformDevState *s)
{
    struct HWGCStageData *d = &s->stageData;

    switch (s->sub_stage)
    {
    case 0:
        d->previous = STAGE_ALLOC;
        d->previous_sub_stage = 1;
        hwgc_goto_stage(s, STAGE_ALLOCATE_DIRECT, 0);
        break;
    case 1:
        if (d->to_obj == 0)
        {
            if (!hwgc_access(s, d->pars.plabAllocatorPtr + 0x18, &d->buffer_temp, 8, false))
                return;
            s->sub_stage = 2;
        }
        else
        {
            s->sub_stage = 5;
        }
        break;
    case 2:
        if (!hwgc_access(s, d->buffer_temp, &d->buffer, 8, false))
            return;
        if (!hwgc_access(s, d->buffer + 0x30, &d->region_top, 8, false))
            return;
        if (!hwgc_access(s, d->buffer + 0x38, &d->region_end, 8, false))
            return;
        s->sub_stage = 3;
        break;
    case 3:
        d->dest_attr = (d->dest_attr & 0x00ff) | 0x0100;
        if (d->region_end >= d->region_top && (d->region_end - d->region_top) / 8 >= d->size)
        {
            d->to_obj = d->region_top;
            uintptr_t write_top = d->to_obj + d->size * 8;
            if (!hwgc_access(s, d->buffer + 0x30, &write_top, 8, true))
                return;
            d->region_top = write_top;
            s->sub_stage = 4;
        }
        else
        {
            d->previous = STAGE_ALLOC;
            d->previous_sub_stage = 4;
            hwgc_goto_stage(s, STAGE_ALLOCATE_DIRECT, 0);
        }
        break;
    case 4:
    {
        uint8_t destination_full = 1;
        if (!hwgc_access(s, d->dest_attr_ptr + 1, &destination_full, 1, true))
            return;
        s->sub_stage = 5;
        break;
    }
    case 5:
        hwgc_goto_stage(s, STAGE_COPY2SURVIVOR, 8);
        break;
    default:
        hwgc_goto_stage(s, STAGE_ALLOC, 0);
        break;
    }
}

static void stage_allocate_direct_function(HWGCPlatformDevState *s)
{
    struct HWGCStageData *d = &s->stageData;
    int8_t dest_attr_type = (int8_t)(d->dest_attr >> 8);

    switch (s->sub_stage)
    {
    case 0:
    {
        uintptr_t plab_stats_ptr = d->pars.g1h + (dest_attr_type == 1 ? 0x2e0 : 0x250);
        if (!hwgc_access(s, plab_stats_ptr + 0x30, &d->region_attr_ptr, 8, false))
            return;
        d->plab_refill_failed = false;
        s->sub_stage = 1;
        break;
    }
    case 1:
        d->plab_word_size = MIN(MAX(d->region_attr_ptr / 2, 0x102), 0x40000);
        d->required_in_plab = d->size + 0x2;
        if (!hwgc_access(s, d->pars.plabAllocatorPtr + 0x8, &d->allocator_ptr, 8, false))
            return;
        s->sub_stage = 2;
        break;
    case 2:
    {
        bool may_throw_away_buffer = d->required_in_plab * 100 < d->plab_word_size * 0xa;
        if (d->required_in_plab <= d->plab_word_size && may_throw_away_buffer)
        {
            uintptr_t buffer_slot = d->pars.plabAllocatorPtr + 0x10 + dest_attr_type * 8;
            if (!hwgc_access(s, buffer_slot, &d->buffer_temp, 8, false))
                return;
            s->sub_stage = 3;
        }
        else
        {
            s->sub_stage = 7;
        }
        break;
    }
    case 3:
        if (!hwgc_access(s, d->buffer_temp, &d->buffer, 8, false))
            return;
        if (!hwgc_access(s, d->buffer + 0x30, &d->region_top, 8, false))
            return;
        if (!hwgc_access(s, d->buffer + 0x40, &d->region_hard_end, 8, false))
            return;
        s->sub_stage = 4;
        break;
    case 4:
        if (d->region_top < d->region_hard_end)
        {
            size_t words = (d->region_hard_end - d->region_top) / 8;
            uintptr_t cur_klass = 0;
            uint header_words = d->pars.useCompressedKlassPointers ? 2 : 3;
            if (words >= header_words)
            {
                uint payload_size = words - header_words;
                uint32_t len = payload_size * 2;
                uintptr_t array_len_addr = d->region_top + (d->pars.useCompressedKlassPointers ? 12 : 16);
                if (!hwgc_access(s, array_len_addr, &len, 4, true))
                    return;
                cur_klass = d->pars.intArrayKlassObj;
            }
            else if (words > 0)
            {
                cur_klass = d->pars.objectKlass;
            }
            uintptr_t mark = 0x1;
            if (!hwgc_access(s, d->region_top, &mark, 8, true))
                return;
            if (d->pars.useCompressedKlassPointers)
            {
                uint32_t narrow_klass = (uint32_t)((cur_klass - s->stageData.pars.compressedKlassPointerBase) >> s->stageData.pars.compressedKlassPointerShift);
                if (!hwgc_access(s, d->region_top + 0x8, &narrow_klass, 4, true))
                    return;
            }
            else
            {
                if (!hwgc_access(s, d->region_top + 0x8, &cur_klass, 8, true))
                    return;
            }
        }
        s->sub_stage = 5;
        break;
    case 5:
        d->min_word_size = d->required_in_plab;
        d->desired_word_size = d->plab_word_size;
        d->during_gc_select = 0;
        hwgc_goto_stage(s, STAGE_ALLOCATE_DURING_GC, 0);
        break;
    case 6:
        if (d->to_obj != 0)
        {
            uintptr_t write30 = (d->actual_plab_size - 2) >= d->size ? d->to_obj + d->size * 8 : d->to_obj;
            uintptr_t write38 = d->to_obj + (d->actual_plab_size - 2) * 8;
            uintptr_t write40 = d->to_obj + d->actual_plab_size * 8;
            uintptr_t write48;
            if (!hwgc_access(s, d->buffer + 0x48, &write48, 8, false))
                return;
            write48 += d->actual_plab_size;
            if (!hwgc_access(s, d->buffer + 0x20, &d->actual_plab_size, 8, true))
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
            if (d->actual_plab_size - 2 < d->size)
                d->to_obj = 0;
            hwgc_return_previous(s);
        }
        else if (d->region_top < d->region_hard_end)
        {
            if (!hwgc_access(s, d->buffer + 0x38, &d->region_hard_end, 8, true))
                return;
            if (!hwgc_access(s, d->buffer + 0x30, &d->region_hard_end, 8, true))
                return;
            if (!hwgc_access(s, d->buffer + 0x28, &d->region_hard_end, 8, true))
                return;
            d->plab_refill_failed = true;
            s->sub_stage = 7;
        }
        else
        {
            s->sub_stage = 7;
        }
        break;
    case 7:
        d->min_word_size = d->size;
        d->desired_word_size = d->size;
        d->during_gc_select = 1;
        hwgc_goto_stage(s, STAGE_ALLOCATE_DURING_GC, 0);
        break;
    case 8:
        hwgc_return_previous(s);
        break;
    default:
        hwgc_goto_stage(s, STAGE_ALLOCATE_DIRECT, 0);
        break;
    }
}

static void stage_allocate_during_gc_function(HWGCPlatformDevState *s)
{
    struct HWGCStageData *d = &s->stageData;
    int8_t dest_attr_type = (int8_t)(d->dest_attr >> 8);
    uint expected = 0;
    uint writed = 1;
    uint get;
    uintptr_t lock_ptr;

    switch (s->sub_stage)
    {
    case 0:
        if (dest_attr_type == 0)
        {
            if (!hwgc_access(s, d->allocator_ptr + 0x28, &d->region_ptr, 8, false))
                return;
        }
        else
            d->region_ptr = d->allocator_ptr + 0x30;
        s->sub_stage = 1;
        break;
    case 1:
        if (!hwgc_access(s, d->region_ptr + 0x8, &d->alloc_region, 8, false))
            return;
        s->sub_stage = 2;
        break;
    case 2:
        if (dest_attr_type == 0)
        {
            d->par_alloc_iml_sel = 0;
            hwgc_goto_stage(s, STAGE_PAR_ALLOCATE_IML, 0);
        }
        else
        {
            lock_ptr = d->alloc_region + 0x40;
            if (!hwgc_cmpxchg(s, lock_ptr + 8, expected, writed, 4, &get))
                return;
            if (get == expected)
            {
                d->par_alloc_sel = 0;
                d->bot_updates = true;
                hwgc_goto_stage(s, STAGE_PAR_ALLOCATE, 0);
            }
            else
            {
                s->sub_stage = 3;
            }
        }
        break;
    case 3:
        lock_ptr = d->alloc_region + 0x40;
        expected = 1;
        writed = 0;
        if (!hwgc_cmpxchg(s, lock_ptr + 8, expected, writed, 4, &get))
            return;
        if (get > 1)
        {
            s->irq_par0 = lock_ptr;
            s->timer_running = false;
            s->irq_to_sub_stage = 4;
            qatomic_or(&s->status, ST_WAIT_WAKE);
            hwgc_raise_irq_from_worker(s, IRQ_WAKE);
            break;
        }
        else
            s->sub_stage = 4;
        break;
    case 4:
        if (d->to_obj == 0)
        {
            d->region_attr_ptr = 0;
            if (!hwgc_access(s, d->allocator_ptr + 0x10, &d->region_attr_ptr, 1, false))
                return;
            s->sub_stage = 6;
        }
        else
            s->sub_stage = 5;
        break;
    case 5:
        hwgc_goto_stage(s, STAGE_ALLOCATE_DIRECT, d->during_gc_select ? 8 : 6);
        break;
    case 6:
    {
        bool is_full = dest_attr_type == 0 ? d->region_attr_ptr & 0x1 : d->region_attr_ptr & 0x2;
        if (!is_full)
        {
            if (dest_attr_type == 0)
            {
                d->par_alloc_iml_sel = 1;
                hwgc_goto_stage(s, STAGE_PAR_ALLOCATE_IML, 0);
            }
            else
            {
                lock_ptr = d->alloc_region + 0x40;
                expected = 0;
                writed = 1;
                if (!hwgc_cmpxchg(s, lock_ptr + 8, expected, writed, 4, &get))
                    return;
                if (get == expected)
                {
                    d->par_alloc_sel = 1;
                    d->bot_updates = true;
                    hwgc_goto_stage(s, STAGE_PAR_ALLOCATE, 0);
                }
                else
                {
                    s->sub_stage = 7;
                }
            }
        }
        else
            s->sub_stage = 5;
        break;
    }
    case 7:
        lock_ptr = d->alloc_region + 0x40;
        expected = 1;
        writed = 0;
        if (!hwgc_cmpxchg(s, lock_ptr + 8, expected, writed, 4, &get))
            return;
        if (get > 1)
        {
            s->irq_par0 = lock_ptr;
            s->timer_running = false;
            s->irq_to_sub_stage = 8;
            qatomic_or(&s->status, ST_WAIT_WAKE);
            hwgc_raise_irq_from_worker(s, IRQ_WAKE);
            break;
        }
        else
            s->sub_stage = 8;
        break;
    case 8:
        if (d->to_obj == 0)
        {
            expected = 0;
            writed = 1;
            if (!hwgc_cmpxchg(s, d->pars.lockPtr + 8, expected, writed, 4, &get))
                return;
            if (get == expected)
                s->sub_stage = 9;
        }
        else
            s->sub_stage = 5;
        break;
    case 9:
        if (!hwgc_access(s, d->pars.lockPtr, &d->pars.thread, 8, true))
            return;
        hwgc_goto_stage(s, STAGE_ATTEMPT_ALLOC, 0);
        break;
    case 10:
        if (d->to_obj == 0)
        {
            uintptr_t addr = dest_attr_type == 0 ? d->allocator_ptr + 0x10 : d->allocator_ptr + 0x11;
            uint8_t full = 1;
            if (!hwgc_access(s, addr, &full, 1, true))
                return;
        }
        s->sub_stage = 11;
        break;
    case 11:
        expected = 1;
        writed = 0;
        if (!hwgc_cmpxchg(s, d->pars.lockPtr + 8, expected, writed, 4, &get))
            return;
        if (get > 1)
        {
            s->irq_par0 = d->pars.lockPtr + 8;
            s->timer_running = false;
            s->irq_to_sub_stage = 5;
            qatomic_or(&s->status, ST_WAIT_WAKE);
            hwgc_raise_irq_from_worker(s, IRQ_WAKE);
            break;
        }
        else
            s->sub_stage = 5;
        break;
    default:
        hwgc_goto_stage(s, STAGE_ALLOCATE_DURING_GC, 0);
        break;
    }
}

static void stage_attempt_alloc_function(HWGCPlatformDevState *s)
{
    struct HWGCStageData *d = &s->stageData;
    switch (s->sub_stage)
    {
    case 0:
        if (!hwgc_access(s, d->region_ptr + 0x40, &d->region_ptr_type, 1, false))
            return;
        if (!hwgc_access(s, d->region_ptr + 0x18, &d->region_attr_ptr, 8, false))
            return;
        s->sub_stage = 1;
        break;
    case 1:
        if (d->alloc_region != d->pars.dummyRegion)
        {
            if (!hwgc_access(s, d->alloc_region, &d->alloc_end, 8, false))
                return;
            if (!hwgc_access(s, d->alloc_region + 0x10, &d->alloc_top, 8, false))
                return;
            if (!hwgc_access(s, d->alloc_region + 0xe8, &d->alloc_start, 8, false))
                return;
            d->allocated_bytes = d->alloc_top - d->alloc_end - d->region_attr_ptr;
            s->sub_stage = 2;
        }
        else
            s->sub_stage = 7;
        break;
    case 2:
        if (d->region_ptr_type == 1)
        {
            uintptr_t addr = d->pars.g1h + 0xa0 + 0x10;
            d->region_attr_ptr = 0;
            if (!hwgc_access(s, addr, &d->region_attr_ptr, 4, false))
                return;
            d->region_attr_ptr += 1;
            if (!hwgc_access(s, addr, &d->region_attr_ptr, 4, true))
                return;
        }
        else
        {
            uintptr_t addr = d->pars.g1h + 0x3f8 + 0x10;
            if (!hwgc_access(s, addr, &d->region_attr_ptr, 8, false))
                return;
            d->region_attr_ptr += d->allocated_bytes;
            if (!hwgc_access(s, addr, &d->region_attr_ptr, 8, true))
                return;
        }
        s->sub_stage = 3;
        break;
    case 3:
    {
        bool during_im_cache;
        if (!hwgc_access(s, d->pars.g1h + 0x3c1, &during_im_cache, 1, false))
            return;
        if (during_im_cache && d->allocated_bytes > 0)
        {
            if (!hwgc_access(s, d->pars.g1h + 0x4e8, &d->cm_cache, 8, false))
                return;
            s->sub_stage = 4;
        }
        else
            s->sub_stage = 6;
        break;
    }
    case 4:
        d->root_regions_ptr = d->cm_cache + 0xb0;
        if (!hwgc_access(s, d->root_regions_ptr, &d->root_regions_array, 8, false))
            return;
        if (!hwgc_access(s, d->root_regions_ptr + 0x10, &d->root_regions_idx, 8, false))
            return;
        s->sub_stage = 5;
        break;
    case 5:
    {
        uintptr_t addr = d->root_regions_array + d->root_regions_idx * 0x10;
        if (!hwgc_access(s, addr, &d->alloc_start, 8, true))
            return;
        uintptr_t writeValue = (d->alloc_top - d->alloc_start) >> 3;
        if (!hwgc_access(s, addr + 0x8, &writeValue, 8, true))
            return;
        d->root_regions_idx += 1;
        if (!hwgc_access(s, d->root_regions_ptr + 0x10, &d->root_regions_idx, 8, true))
            return;
        s->sub_stage = 6;
        break;
    }
    case 6:
        d->region_attr_ptr = 0;
        if (!hwgc_access(s, d->region_ptr + 0x18, &d->region_attr_ptr, 8, true))
            return;
        if (!hwgc_access(s, d->region_ptr + 0x8, &d->pars.dummyRegion, 8, true))
            return;
        s->sub_stage = 7;
        break;
    case 7:
        hwgc_goto_stage(s, STAGE_NEW_GC_ALLOC, 0);
        break;
    case 8:
        if (d->new_alloc_region == 0)
        {
            d->to_obj = 0;
            hwgc_goto_stage(s, STAGE_ALLOCATE_DURING_GC, 10);
            return;
        }
        if (!hwgc_access(s, d->region_ptr + 0x20, &d->bot_updates, 1, false))
            return;
        d->alloc_region = d->new_alloc_region;
        d->min_word_size = d->desired_word_size;
        d->par_alloc_sel = 2;
        hwgc_goto_stage(s, STAGE_PAR_ALLOCATE, 0);
        return;
    case 9:
        d->region_attr_ptr = 0;
        if (!hwgc_access(s, d->new_alloc_region + 0xa8, &d->region_attr_ptr, 8, true))
            return;
        if (!hwgc_access(s, d->new_alloc_region, &d->alloc_end, 8, false))
            return;
        if (!hwgc_access(s, d->new_alloc_region + 0x10, &d->alloc_top, 8, false))
            return;
        d->region_attr_ptr = d->alloc_top - d->alloc_end;
        if (!hwgc_access(s, d->region_ptr + 0x18, &d->region_attr_ptr, 8, true))
            return;
        if (!hwgc_access(s, d->region_ptr + 0x8, &d->new_alloc_region, 8, true))
            return;
        d->region_attr_ptr = 0;
        if (!hwgc_access(s, d->region_ptr + 0x10, &d->region_attr_ptr, 4, false))
            return;
        d->region_attr_ptr += 1;
        if (!hwgc_access(s, d->region_ptr + 0x10, &d->region_attr_ptr, 4, true))
            return;
        if (d->to_obj != 0)
            d->actual_plab_size = d->desired_word_size;
        hwgc_goto_stage(s, STAGE_ALLOCATE_DURING_GC, 10);
        break;
    default:
        break;
    }
}

static void stage_new_gc_alloc_function(HWGCPlatformDevState *s)
{
    struct HWGCStageData *d = &s->stageData;

    switch (s->sub_stage)
    {
    case 0:
        if (!hwgc_access(s, d->region_ptr + 0x30, &d->node_index, 4, false))
            return;
        d->heap_region_type = (d->region_ptr_type == 1) ? 0x10 : 0x3;
        d->alloc_free_sel = 0;
        hwgc_goto_stage(s, STAGE_ALLOCATE_FREE, 0);
        s->sub_stage = 1;
        break;
    case 1:
        if (d->new_alloc_region == 0)
        {
            bool expand_failure;
            if (!hwgc_access(s, d->pars.g1h + 0x370, &expand_failure, 1, false))
                return;
            if (expand_failure)
            {
                d->region_attr_ptr = 0;
                s->irq_par0 = d->node_index;
                s->timer_running = false;
                s->irq_to_sub_stage = 2;
                qatomic_or(&s->status, ST_WAIT_EXPAND);
                hwgc_raise_irq_from_worker(s, IRQ_EXPAND);
                return;
            }
            s->sub_stage = 3;
        }
        else
            s->sub_stage = 3;
        break;
    case 2:
        if (d->region_attr_ptr)
        {
            d->alloc_free_sel = 1;
            hwgc_goto_stage(s, STAGE_ALLOCATE_FREE, 0);
        }
        else
        {
            if (!hwgc_access(s, d->pars.g1h + 0x370, &d->region_attr_ptr, 1, true))
                return;
            s->sub_stage = 3;
        }
        break;
    case 3:
        if (d->new_alloc_region == 0)
        {
            hwgc_goto_stage(s, STAGE_ATTEMPT_ALLOC, 8);
            return;
        }
        if (!hwgc_access(s, d->pars.g1h + 0x3f8 + 0x8, &d->grow_array_ptr, 8, false))
            return;
        s->sub_stage = 4;
        break;
    case 4:
        if (!hwgc_access(s, d->new_alloc_region + 0xbc, &d->heap_region_type, 4, true))
            return;
        if (!hwgc_access(s, d->new_alloc_region + 0xb0, &d->remset_ptr, 8, false))
            return;
        if (!hwgc_access(s, d->new_alloc_region + 0xb8, &d->node_index, 4, false))
            return;
        if (d->heap_region_type == 0x3)
            s->sub_stage = 5;
        else
            s->sub_stage = 8;
        break;
    case 5:
        if (!hwgc_access(s, d->grow_array_ptr, &d->region_attr_ptr, 8, false))
            return;
        d->grow_array_len = (uint)d->region_attr_ptr;
        d->grow_array_max = (uint)(d->region_attr_ptr >> 32);
        if (d->grow_array_len == d->grow_array_max)
        {
            s->irq_par0 = d->grow_array_ptr;
            s->irq_par1 = d->grow_array_len;
            s->timer_running = false;
            s->irq_to_sub_stage = 6;
            qatomic_or(&s->status, ST_WAIT_GROW);
            hwgc_raise_irq_from_worker(s, IRQ_GROW);
            return;
        }
        s->sub_stage = 6;
        break;
    case 6:
        d->region_attr_ptr = d->grow_array_len + 1;
        if (!hwgc_access(s, d->grow_array_ptr, &d->region_attr_ptr, 4, true))
            return;
        if (!hwgc_access(s, d->grow_array_ptr + 0x8, &d->data_ptr, 8, false))
            return;
        s->sub_stage = 7;
        break;
    case 7:
    {
        uintptr_t addr = d->data_ptr + ((uintptr_t)d->grow_array_len * 8);
        if (!hwgc_access(s, addr, &d->new_alloc_region, 8, true))
            return;
        s->sub_stage = 8;
        break;
    }
    case 8:
    {
        uintptr_t addr = d->remset_ptr + 0xf0;
        uint writeValue;
        if ((d->heap_region_type & 0x2) != 0)
            writeValue = 2;
        else if ((d->heap_region_type & 0x10) != 0)
            writeValue = 0;
        if (!hwgc_access(s, addr, &writeValue, 4, true))
            return;
        s->sub_stage = 9;
        break;
    }
    case 9:
        if (!hwgc_access(s, d->pars.g1h + 0x580 + 0x10, &d->region_attr_base, 8, false))
            return;
        s->sub_stage = 10;
        break;
    case 10:
    {
        uintptr_t addr = d->region_attr_base + d->node_index * 2;
        uint8_t writeValue = ((d->heap_region_type & 0x10) == 0);
        if (!hwgc_access(s, addr, &writeValue, 1, true))
            return;
        hwgc_goto_stage(s, STAGE_ATTEMPT_ALLOC, 8);
        break;
    default:
        break;
    }
}

static void stage_alloc_free_region_function(HWGCPlatformDevState *s)
{
    struct HWGCStageData *d = &s->stageData;

    switch (s->sub_stage)
    {
    case 0:
        d->hrm_ptr = d->pars.g1h + 0x130;
        d->free_list_ptr = d->hrm_ptr + 0xb0;
        d->from_head = ((d->heap_region_type & 0x2) == 0);
        d->new_alloc_region = 0;
        if (!hwgc_access(s, d->free_list_ptr + 0x10, &d->list_length, 4, false))
            return;
        if (!hwgc_access(s, d->free_list_ptr + 0x28, &d->list_head_ptr, 8, false))
            return;
        if (!hwgc_access(s, d->free_list_ptr + 0x30, &d->list_end_ptr, 8, false))
            return;
        if (!hwgc_access(s, d->free_list_ptr + 0x38, &d->list_last_ptr, 8, false))
            return;
        if (d->list_length == 0)
        {
            hwgc_goto_stage(s, STAGE_NEW_GC_ALLOC, d->alloc_free_sel ? 3 : 1);
            return;
        }
        s->sub_stage = 1;
        break;
    case 1:
        d->new_alloc_region = d->from_head ? d->list_head_ptr : d->list_end_ptr;
        if (!hwgc_access(s, d->new_alloc_region + (d->from_head ? 0xd0 : 0xd8), &d->res_conf, 8, false))
            return;
        if (!hwgc_access(s, d->free_list_ptr + (d->from_head ? 0x28 : 0x30), &d->res_conf, 8, true))
            return;
        s->sub_stage = 2;
        break;
    case 2:
        if (d->res_conf == 0)
        {
            uintptr_t zero = 0;
            if (!hwgc_access(s, d->free_list_ptr + (d->from_head ? 0x30 : 0x28), &zero, 8, true))
                return;
        }
        else
        {
            uintptr_t zero = 0;
            if (!hwgc_access(s, d->res_conf + (d->from_head ? 0xd8 : 0xd0), &zero, 8, true))
                return;
        }
        s->sub_stage = 3;
        break;
    case 3:
    {
        uintptr_t zero = 0;
        if (!hwgc_access(s, d->new_alloc_region + (d->from_head ? 0xd0 : 0xd8), &zero, 8, true))
            return;
        s->sub_stage = 4;
        break;
    }
    case 4:
        if (d->new_alloc_region == 0)
        {
            hwgc_goto_stage(s, STAGE_NEW_GC_ALLOC, d->alloc_free_sel ? 3 : 1);
            return;
        }
        if (d->list_last_ptr == d->new_alloc_region)
        {
            uintptr_t zero = 0;
            if (!hwgc_access(s, d->free_list_ptr + 0x38, &zero, 8, true))
                return;
        }
        s->sub_stage = 5;
        break;
    case 5:
    {
        uint writeValue = d->list_length - 1;
        if (!hwgc_access(s, d->free_list_ptr + 0x10, &writeValue, 4, true))
            return;
        hwgc_goto_stage(s, STAGE_NEW_GC_ALLOC, d->alloc_free_sel ? 3 : 1);
        break;
    }
    default:
        break;
    }
}

static void stage_alloc_free_region_function(HWGCPlatformDevState *s)
{
    struct HWGCStageData *d = &s->stageData;

    switch (s->sub_stage)
    {
    case 0:
        if (!hwgc_access(s, d->pars.g1h + 0x3f8, &d->remaining, 8, false))
            return;
        s->sub_stage = 1;
        break;
    case 1:
        if (d->remaining == 0)
        {
            d->new_alloc_region = 0;
            hwgc_return_previous(s);
            return;
        }
        if (!hwgc_access(s, d->remaining + 0x10, &d->offset30, 8, false))
            return;
        if (!hwgc_access(s, d->remaining + 0x18, &d->offset38, 8, false))
            return;
        s->sub_stage = 2;
        break;
    case 2:
        if (d->offset30 >= d->offset38)
        {
            hwgc_goto_stage(s, STAGE_PAR_ALLOCATE_IML, 0);
            return;
        }
        d->new_alloc_region = d->remaining;
        hwgc_return_previous(s);
        break;
    default:
        hwgc_return_previous(s);
        break;
    }
}

static void stage_par_allocate_iml_function(HWGCPlatformDevState *s)
{
    struct HWGCStageData *d = &s->stageData;

    switch (s->sub_stage)
    {
    case 0:
        if (!hwgc_access(s, d->pars.g1h + 0x3f8, &d->remaining, 8, false))
            return;
        s->sub_stage = 1;
        break;
    case 1:
        if (d->remaining == 0)
        {
            hwgc_return_previous(s);
            return;
        }
        if (!hwgc_access(s, d->remaining + 0x10, &d->alloc_top, 8, false))
            return;
        if (!hwgc_access(s, d->remaining + 0x18, &d->alloc_end, 8, false))
            return;
        s->sub_stage = 2;
        break;
    case 2:
        d->desired_word_size = MAX(d->desired_word_size, d->min_word_size);
        if (d->alloc_top > d->alloc_end && (d->alloc_top - d->alloc_end) / 8 >= d->desired_word_size)
        {
            d->to_obj = d->alloc_end;
            hwgc_return_previous(s);
        }
        else
        {
            hwgc_goto_stage(s, STAGE_PAR_ALLOCATE, 0);
        }
        break;
    default:
        hwgc_return_previous(s);
        break;
    }
}

static void stage_par_allocate_function(HWGCPlatformDevState *s)
{
    struct HWGCStageData *d = &s->stageData;

    switch (s->sub_stage)
    {
    case 0:
        if (!hwgc_access(s, d->alloc_region + 0x20, &d->offset30, 8, false))
            return;
        if (!hwgc_access(s, d->alloc_region + 0x28, &d->offset38, 8, false))
            return;
        s->sub_stage = 1;
        break;
    case 1:
        if (d->offset30 < d->offset38 && (d->offset38 - d->offset30) / 8 >= d->desired_word_size)
        {
            d->to_obj = d->offset30;
            uintptr_t new_top = d->offset30 + d->desired_word_size * 8;
            if (!hwgc_access(s, d->alloc_region + 0x20, &new_top, 8, true))
                return;
            hwgc_return_previous(s);
        }
        else
        {
            hwgc_goto_stage(s, STAGE_COPY, 0);
        }
        break;
    default:
        hwgc_return_previous(s);
        break;
    }
}

static void stage_copy_function(HWGCPlatformDevState *s)
{
    struct HWGCStageData *d = &s->stageData;
    switch (s->sub_stage)
    {
    case 0:
        if (d->i >= d->size)
        {
            hwgc_goto_stage(s, STAGE_COPY2SURVIVOR, 12);
            return;
        }
        if (!hwgc_access(s, d->from_obj + d->i * 8, &d->region_attr_ptr, 8, false))
            return;
        s->sub_stage = 1;
        break;
    case 1:
        if (!hwgc_access(s, d->to_obj + d->i * 8, &d->region_attr_ptr, 8, true))
            return;
        d->i++;
        s->sub_stage = 0;
        break;
    default:
        hwgc_goto_stage(s, STAGE_COPY, 0);
        break;
    }
}

static void stage_trace_function(HWGCPlatformDevState *s)
{
    struct HWGCStageData *d = &s->stageData;
    switch (s->sub_stage)
    {
    case 0:
        d->src = d->p - d->to_obj + d->from_obj;
        d->dest = d->p;
        d->p += (d->pars.useCompressedOops ? 4 : 8);
        hwgc_goto_stage(s, STAGE_DO_OOP_WORK, 0);
        break;
    default:
        hwgc_return_done(s);
        break;
    }
}

static void stage_trace_plus_function(HWGCPlatformDevState *s)
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
    {
        hwgc_return_done(s);
    }
}

static void stage_trace_dec_function(HWGCPlatformDevState *s)
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
    {
        hwgc_return_done(s);
    }
}

static void stage_do_oop_work_function(HWGCPlatformDevState *s)
{
    struct HWGCStageData *d = &s->stageData;

    switch (s->sub_stage)
    {
    case 0:
        d->heap_oop = 0;
        if (!hwgc_access(s, d->src, &d->heap_oop, d->pars.useCompressedOops ? 4 : 8, false))
            return;
        s->sub_stage = 1;
        break;
    case 1:
        if (d->heap_oop == 0)
        {
            s->sub_stage = 7;
            break;
        }
        if (d->pars.useCompressedOops)
            d->heap_oop = d->pars.compressedOopBase + ((uint32_t)d->heap_oop << d->pars.compressedOopShift);
        d->region_attr_ptr = d->pars.regionAttrBiasedBase + (d->heap_oop >> d->pars.regionAttrShiftBy) * 2;
        if (!hwgc_access(s, d->region_attr_ptr, &d->region_attr_ptr, 2, false))
            return;
        s->sub_stage = 2;
        break;
    case 2:
    {
        int8_t region_attr_type = (int8_t)(d->region_attr_ptr >> 8);
        bool cross_region = ((d->dest ^ d->heap_oop) >> d->pars.logOfHRGrainBytes) != 0;
        if (region_attr_type >= 0)
        {
            uintptr_t writeElems = d->dest + (d->pars.useCompressedOops ? 1 : 0);
            uintptr_t queue_addr = d->pars.taskQueueElemsBase + d->localBot * 8;
            if (!hwgc_access(s, queue_addr, &writeElems, 8, true))
                return;
            d->localBot = (d->localBot + 1) & ((1 << 17) - 1);
            s->sub_stage = 7;
        }
        else if (cross_region)
        {
            if (region_attr_type == -2)
                s->sub_stage = 3;
            else
                s->sub_stage = 6;
        }
        else
        {
            s->sub_stage = 7;
        }
        break;
    }
    case 3:
        if (!hwgc_access(s, d->pars.humogousReclaimCandidateBoolBase + d->region, &d->bool_base_value, 1, false))
            return;
        s->sub_stage = 4;
        break;
    case 4:
        if (!d->bool_base_value)
        {
            s->sub_stage = 6;
            break;
        }
        d->bool_base_value = false;
        if (!hwgc_access(s, d->pars.humogousReclaimCandidateBoolBase + d->region, &d->bool_base_value, 1, true))
            return;
        s->sub_stage = 5;
        break;
    case 5:
    {
        uintptr_t region_attr_dest = d->pars.regionAttrBase + d->region * 2;
        int8_t dest_value = -1;
        if (!hwgc_access(s, region_attr_dest + 1, &dest_value, 1, true))
            return;
        s->sub_stage = 6;
        break;
    }
    case 6:
        if (d->scanning_in_young)
            hwgc_return_previous(s);
        else
        {
            d->aop_region_attr = (uint16_t)d->region_attr_ptr;
            d->aop_dest = d->dest;
            hwgc_goto_stage(s, STAGE_AOP_WORK, 0);
        }
        break;
    case 7:
        hwgc_return_previous(s);
        break;
    default:
        hwgc_goto_stage(s, STAGE_DO_OOP_WORK, 0);
        break;
    }
}

static void stage_aop_work_function(HWGCPlatformDevState *s)
{
    struct HWGCStageData *d = &s->stageData;

    switch (s->sub_stage)
    {
    case 0:
        if ((d->aop_region_attr & 0xff) == 0)
        {
            hwgc_return_previous(s);
            return;
        }
        if (!hwgc_access(s, d->pars.cardTablePtr + 0x38, &d->byte_map, 8, false))
            return;
        s->sub_stage = 1;
        break;
    case 1:
        if (!hwgc_access(s, d->pars.cardTablePtr + 0x40, &d->res, 8, false))
            return;
        s->sub_stage = 2;
        break;
    case 2:
        if (d->aop_dest == 0)
        {
            hwgc_return_previous(s);
            return;
        }
        hwgc_return_previous(s);
        break;
    default:
        hwgc_return_previous(s);
        break;
    }
}

static uint64_t hwgc_platform_read(void *opaque, hwaddr addr, unsigned size)
{
    HWGCPlatformDevState *s = opaque;
    uint64_t value = 0;

    switch (addr)
    {
    case REG_STATUS:
        value = qatomic_read(&s->status);
        break;
    case REG_IRQ_STATUS:
        value = qatomic_read(&s->irq_status);
        break;
    case REG_IRQ_CLEAR:
    case REG_CMD:
        value = 0;
        break;
    case REG_IRQ_PAR0:
        value = s->irq_par0;
        break;
    case REG_IRQ_PAR1:
        value = s->irq_par1;
        break;
    case REG_IRQ_RES0:
        value = s->irq_res0;
        break;
    case REG_IRQ_RES1:
        value = s->irq_res1;
        break;
    default:
        if (addr >= REG_PAR0 && addr <= REG_PAR21)
        {
            uintptr_t *base = (uintptr_t *)&s->stageData.pars;
            value = base[(addr - REG_PAR0) / 8];
        }
        break;
    }

    return value;
}

static void hwgc_platform_write(void *opaque, hwaddr addr, uint64_t data, unsigned size)
{
    HWGCPlatformDevState *s = opaque;

    switch (addr)
    {
    case REG_IRQ_CLEAR:
        hwgc_lower_irq_from_mmio(s, (uint32_t)data);
        break;
    case REG_CMD:
    {
        bool need_wakeup = false;

        if (data & CMD_RESET)
        {
            qemu_mutex_lock(&s->lock);
            hwgc_reset_device_locked(s);
            qemu_mutex_unlock(&s->lock);
        }
        if (data & CMD_START)
        {
            qemu_mutex_lock(&s->lock);
            qatomic_or(&s->status, ST_BUSY);
            s->stage = STAGE_FETCH;
            s->sub_stage = 0;
            s->timer_running = true;
            need_wakeup = true;
            qemu_mutex_unlock(&s->lock);
        }
        if (data & CMD_CONTINUE)
        {
            qemu_mutex_lock(&s->lock);
            s->timer_running = true;
            need_wakeup = true;
            qemu_mutex_unlock(&s->lock);
        }
        if (need_wakeup)
        {
            hwgc_worker_wakeup(s);
        }
        break;
    }
    case REG_IRQ_PAR0:
        s->irq_par0 = data;
        break;
    case REG_IRQ_PAR1:
        s->irq_par1 = data;
        break;
    case REG_IRQ_RES0:
        s->irq_res0 = data;
        break;
    case REG_IRQ_RES1:
        s->irq_res1 = data;
        break;
    default:
        if (addr >= REG_PAR0 && addr <= REG_PAR21)
        {
            uintptr_t *base = (uintptr_t *)&s->stageData.pars;
            base[(addr - REG_PAR0) / 8] = data;
        }
        break;
    }
}

static const MemoryRegionOps hwgc_platform_ops = {
    .read = hwgc_platform_read,
    .write = hwgc_platform_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 8,
    .impl.min_access_size = 1,
    .impl.max_access_size = 8,
};

static void hwgc_worker_wakeup(void *opaque)
{
    HWGCPlatformDevState *s = opaque;
    qemu_mutex_lock(&s->lock);
    s->tick_pending = true;
    qemu_cond_signal(&s->cond);
    qemu_mutex_unlock(&s->lock);
}

static void hwgc_timer_cb(void *opaque)
{
    HWGCPlatformDevState *s = opaque;
    qemu_mutex_lock(&s->lock);
    if (!s->thread_stop && s->timer_running)
    {
        s->tick_pending = true;
        qemu_cond_signal(&s->cond);
    }
    qemu_mutex_unlock(&s->lock);
}

static void *hwgc_worker_thread(void *opaque)
{
    HWGCPlatformDevState *s = opaque;

    while (true)
    {
        qemu_mutex_lock(&s->lock);

        while (!s->tick_pending && !s->thread_stop)
        {
            qemu_cond_wait(&s->cond, &s->lock);
        }

        if (s->thread_stop)
        {
            qemu_mutex_unlock(&s->lock);
            break;
        }

        s->tick_pending = false;
        qemu_mutex_unlock(&s->lock);

        if (qatomic_read(&s->status) & ST_BUSY)
        {
            qemu_mutex_lock(&s->lock);
            if (s->timer_running)
            {
                hwgc_platform_stage_dispatch(s);
                if (s->timer_running)
                {
                    hwgc_arm_timer(s);
                }
            }
            qemu_mutex_unlock(&s->lock);
        }
    }

    return NULL;
}

static void hwgc_reset_device_locked(HWGCPlatformDevState *s)
{
    s->timer_running = false;
    s->tick_pending = false;
    qatomic_set(&s->status, ST_IRQ_EN);
    qatomic_set(&s->irq_status, 0);
    s->stage = STAGE_IDLE;
    s->sub_stage = 0;
    s->irq_source = 0;
    s->irq_to_sub_stage = 0;
    s->irq_par0 = 0;
    s->irq_par1 = 0;
    s->irq_res0 = 0;
    s->irq_res1 = 0;
    memset(&s->stageData, 0, sizeof(s->stageData));
    hwgc_tlb_flush(s);
}

static void hwgc_platform_reset(DeviceState *dev)
{
    HWGCPlatformDevState *s = HWGC_PLATFORM_DEV(dev);
    qemu_mutex_lock(&s->lock);
    hwgc_reset_device_locked(s);
    s->thread_stop = false;
    qemu_mutex_unlock(&s->lock);
    s->period_ns = 1000000;
}

static void hwgc_platform_init(Object *obj)
{
    HWGCPlatformDevState *s = HWGC_PLATFORM_DEV(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    sysbus_init_mmio(sbd, &s->mmio);
    sysbus_init_irq(sbd, &s->irq);

    qemu_mutex_init(&s->lock);
    qemu_cond_init(&s->cond);
    s->tick_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, hwgc_timer_cb, s);
    qemu_thread_create(&s->worker, "hwgc-platform", hwgc_worker_thread, s, QEMU_THREAD_JOINABLE);
}

static void hwgc_platform_finalize(Object *obj)
{
    HWGCPlatformDevState *s = HWGC_PLATFORM_DEV(obj);

    qemu_mutex_lock(&s->lock);
    s->thread_stop = true;
    qemu_cond_signal(&s->cond);
    qemu_mutex_unlock(&s->lock);
    qemu_thread_join(&s->worker);
    timer_free(s->tick_timer);
}

static void hwgc_platform_realize(DeviceState *dev, Error **errp)
{
    HWGCPlatformDevState *s = HWGC_PLATFORM_DEV(dev);

    memory_region_init_io(&s->mmio, OBJECT(s), &hwgc_platform_ops, s,
                          TYPE_HWGC_PLATFORM_DEV, 0x1000);
    hwgc_platform_reset(dev);
}

static void hwgc_platform_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = hwgc_platform_realize;
}

static const TypeInfo hwgc_platform_info = {
    .name = TYPE_HWGC_PLATFORM_DEV,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(HWGCPlatformDevState),
    .instance_init = hwgc_platform_init,
    .instance_finalize = hwgc_platform_finalize,
    .class_init = hwgc_platform_class_init,
    .class_size = sizeof(DeviceClass),
};

static void hwgc_platform_register_types(void)
{
    type_register_static(&hwgc_platform_info);
}

type_init(hwgc_platform_register_types)
