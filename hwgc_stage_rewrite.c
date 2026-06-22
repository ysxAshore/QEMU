
/*
 * hwgc_stage_rewrite.c
 *
 * This file rewrites the original monolithic do_hwgc_work() state machine into
 * stage functions of the form:
 *
 *     static void stage_xxx_function(HWGCDevState *s)
 *
 * Assumptions:
 *   1. HWGCDevState already contains:
 *        int stage;
 *        int sub_stage;
 *        int wake_stage;
 *        int wake_sub_stage;
 *        HWGCStageData stageData;
 *        atomic/status fields used by qatomic_read(&s->status)
 *
 *   2. Existing helper:
 *        bool hwgc_access(HWGCDevState *s,
 *                         uintptr_t addr,
 *                         void *data,
 *                         size_t size,
 *                         bool is_write);
 *
 *      It replaces TRY_R/TRY_W/safeAccessHWAddr style calls.
 *
 *   3. Existing enums/macros:
 *        STAGE_FETCH, STAGE_DISPATCH, STAGE_PARTIAL_ARRAY,
 *        STAGE_COMMON_OOP, STAGE_COPY2SURVIVOR, STAGE_ALLOC,
 *        STAGE_ALLOCATE_DIRECT, STAGE_ALLOCATE_DURING_GC,
 *        STAGE_PAR_ALLOCATE_IML, STAGE_PAR_ALLOCATE,
 *        STAGE_ATTEMPT_ALLOC, STAGE_NEW_GC_ALLOC,
 *        STAGE_ALLOC_FREE_REGION, STAGE_COPY, STAGE_TRACE,
 *        STAGE_TRACE_PLUS, STAGE_TRACE_DEC, STAGE_DO_OOP_WORK,
 *        STAGE_AOP_WORK, STAGE_DEBUG, STAGE_DONE
 *
 *   4. Existing IRQ helpers / globals:
 *        qatomic_read, bql_lock, bql_unlock, hwgc_raise_irq
 *        HWGC_STATUS_IRQ, DEBUG_IRQ, ALLOC_SLOW_IRQ, ENQUEUE_FAILED_IRQ
 *        alloc_irq, grow_irq, enqueued_irq
 *
 *   5. Existing HWGCPars and HWGCSoftPars definitions.
 *
 * Important:
 *   - All original static local variables are moved into HWGCStageData.
 *   - This avoids cross-device/cross-invocation pollution.
 *   - Some same-page reads/writes are intentionally combined.
 *   - Read-modify-write counters are kept in separate cases to avoid
 *     accidental double increments after a retry.
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <limits.h>

#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif

#ifndef MAX
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#endif

/*
 * Move the original static variables here.
 * If your project already has HWGCStageData, merge these fields into it.
 */
typedef struct HWGCStageData {
    HWGCPars pars;
    HWGCSoftPars softPars;

    uintptr_t previous;
    int previous_sub_stage;

    uintptr_t done_to;
    int doneto_sub_stage;

    uintptr_t task;
    uint32_t localBot;

    uintptr_t from_obj;
    uintptr_t to_obj;
    uintptr_t partial_m_value;
    int partial_from_length;
    int start;

    uintptr_t heap_region;
    uint32_t heap_region_type;
    uint32_t ncreate;
    uint32_t i;
    bool scanning_in_young;

    uintptr_t p;
    uintptr_t q;
    uintptr_t src;
    uintptr_t dest;

    uint32_t array_localBot;

    uintptr_t common_m_value;
    uintptr_t src_region_attr_ptr;
    uint16_t src_region_attr;
    uintptr_t originValue;
    uintptr_t offset;
    uint16_t region_attr;
    uintptr_t region_attr_ptr;
    

    uintptr_t klass_ptr;
    size_t size;
    int lh;
    int kid;
    int common_oop_array_length;

    uint16_t copy2survivor_region_attr;
    uint16_t age;
    uint16_t dest_attr;

    uintptr_t dest_attr_ptr;
    uintptr_t monitor_markWord;
    uintptr_t from_region;

    uintptr_t buffer_temp;
    uintptr_t buffer;
    uintptr_t region_top;
    uintptr_t region_end;

    uint32_t young_index;
    uintptr_t new_mark;
    uintptr_t writeSrcMW;

    int8_t dest_attr_type;

    uintptr_t plab_stats_ptr;
    uintptr_t allocator_ptr;
    uintptr_t alloc_klass_ptr;

    size_t plab_word_size;
    size_t required_in_plab;
    size_t actual_plab_size;
    size_t min_word_size;
    size_t desired_word_size;
    int during_gc_select;

    uintptr_t region_ptr;
    uintptr_t alloc_region;

    uintptr_t card_table_ptr;
    uintptr_t byte_map_base;
    uintptr_t first;
    uintptr_t last;

    int par_alloc_iml_sel;
    int par_alloc_sel;
    bool bot_updates;

    uintptr_t alloc_top;
    uintptr_t alloc_end;
    size_t want_to_allocate;

    uintptr_t blk_start;
    uintptr_t blk_end;
    uintptr_t bot_part_ptr;
    uintptr_t bot_ptr;
    uintptr_t next_offset_threshold;
    uintptr_t array;
    uintptr_t reserved_start;
    uintptr_t begin;

    size_t index;
    size_t start_card_for_region;
    size_t start_card;
    size_t end_card;
    size_t reach;
    size_t num_cards;
    uint8_t ct_offset;

    size_t allocated_bytes;
    int8_t type;
    uintptr_t new_alloc_region;
    uintptr_t cm;
    uintptr_t root_regions_array;
    uintptr_t mem_region;
    uintptr_t next_top;

    uint32_t region_node_index;
    uint32_t array_len;
    uint32_t array_max;

    uintptr_t policy_ptr;
    uintptr_t grow_array_ptr;
    uintptr_t data_ptr;
    uintptr_t count_per_node;
    uintptr_t numa;

    bool expand_failure;
    bool allocate_free_sel;

    bool from_head;
    uint32_t active_node_ids;
    uint32_t region_size;
    uint32_t page_size;
    uint32_t cur_depth;
    uint32_t max_depth;

    uintptr_t free_list_ptr;
    uintptr_t cur;
    uintptr_t prev;
    uintptr_t next;

    uintptr_t data;

    int end;
    int vtable_len;
    int staticCount;

    uintptr_t start_map;
    uintptr_t end_map;

    uintptr_t heap_oop;
    uint32_t region;
    bool bool_base_value;

    uintptr_t byte_map;
    uintptr_t res;
    size_t card_index;
    size_t last_index;

    uintptr_t node_allocator_ptr;
    uintptr_t node;
    uintptr_t old_node;
    uintptr_t new_top;

    uint16_t aop_region_attr;
    uintptr_t aop_region_attr_ptr;
    uintptr_t aop_dest;
} HWGCStageData;

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
    HWGCStageData *d = &s->stageData;

    if (d->localBot == 0)
    {
        hwgc_goto_stage(s, STAGE_DONE, 0);
        return;
    }

    uint elems_bias = (d->localBot - 1) & ((1 << 17) - 1);
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
}

static void stage_partial_array_function(HWGCDevState *s)
{
    HWGCStageData *d = &s->stageData;

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
    {
        if (d->i >= d->ncreate)
        {
            s->sub_stage = 4;
            break;
        }

        uintptr_t pushData = pushData = d->from_obj + 0x2;
        if (!hwgc_access(s, d->pars.taskQueueElemsBase + d->localBot * 8, &pushData, 8, true))
            return;

        d->localBot = (d->localBot + 1) & ((1 << 17) - 1);
        d->i++;
        break;
    }

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

static void stage_oop_function(HWGCDevState *s)
{
    HWGCStageData *d = &s->stageData;

    switch (s->sub_stage)
    {
    case 0:
        if (!hwgc_access(s, d->task, &d->offset, 8, false))
            return;

        s->sub_stage = 1;
        break;

    case 1:
        if (d->pars.useCompressedOops)
        {
            uint32_t narrow_oop = (uint32_t)d->offset;
            if (narrow_oop == 0)
                d->from_obj = 0;
            else
                d->from_obj = (uintptr_t)d->pars.compressedOopBase + ((uintptr_t)narrow_oop << d->pars.compressedOopShift);
        }
        else
            d->from_obj = d->offset;

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
            uint32_t narrow_writeObj;

            writeObj = (d->to_obj - d->pars.compressedOopBase) >> d->pars.compressedOopShift;
            narrow_writeObj = (uint32_t)writeObj;
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
    {
        uintptr_t heap_region_ptr;

        if (((d->task ^ d->to_obj) >> d->pars.logOfHRGrainBytes) == 0)
        {
            hwgc_goto_stage(s, STAGE_FETCH, 0);
            return;
        }

        heap_region_ptr = d->pars.heapRegionBiasedBase + (d->task >> d->pars.heapRegionShiftBy) * 8;

        if (!hwgc_access(s, heap_region_ptr, &d->heap_region, 8, false))
            return;

        s->sub_stage = 6;
        break;
    }

    case 6:
        if (!hwgc_access(s, d->heap_region + 0xbc, &d->heap_region_type, 4, false))
            return;

        if ((d->heap_region_type & 0x2) != 0)
        {
            hwgc_goto_stage(s, STAGE_FETCH, 0);
            return;
        }

        uintptr_t attr_ptr = d->pars.regionAttrBiasedBase + (d->to_obj >> d->pars.regionAttrShiftBy) * 2;

        if (!hwgc_access(s, attr_ptr, &d->aop_region_attr, 2, false))
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

static void stage_copy2survivor_function(HWGCDevState *s)
{
    HWGCStageData *d = &s->stageData;

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
        else if(d->lh < 0)
        {
            uintptr_t len_addr = d->from_obj + (d->pars.useCompressedKlassPointers ? 12 : 16);
            if (!hwgc_access(s, len_addr, &d->common_oop_array_length, 4, false))
                return;
            size_t temp = ((size_t)d->common_oop_array_length << (uint8_t)d->lh) + (uint8_t)(d->lh >> 16);
            d->size = (temp & 0x7) ? ((temp >> 3) + 1) : (temp >> 3);
        }else
        {
            if(d->kid == 2)
            {
              if(!hwgc_access(s, d->from_obj + 0x20, &d->size, 4, false))
                  return;
            }else
              d->size = (size_t)d->lh >> 3;
        }
        
        s->sub_stage = 3;
        break;
    
    case 3:
        if (!s->stageData.dest_attr_valid)
        {
            if (!hwgc_access(s, s->stageData.pars.pss + 0x178, &s->stageData.dest_attr_cache, 4, false))
                return;
            s->stageData.dest_attr_valid = true;
        }
        s->stageData.src_region_attr_type = (int8_t)(s->stageData.src_region_attr >> 8);
        s->stageData.dest_attr = s->stageData.src_region_attr_type == 1 ? s->stageData.dest_attr_cache >> 16 : s->stageData.dest_attr_cache & 0xffff;
        s->stageData.dest_attr_ptr = s->stageData.src_region_attr_type == 1 ? s->stageData.pars.pss + 0x178 + 0x2 : s->stageData.pars.pss + 0x178;
        s->sub_stage = 4;
        break;

    case 4:
        if (s->stageData.src_region_attr_type == 0)
        {
            if ((s->stageData.common_m_value & 0x1) == 0x0)
            {
                uintptr_t ptr = (s->stageData.common_m_value & 0x2) ? (s->stageData.common_m_value ^ 0x2) : s->stageData.common_m_value;

                if (!hwgc_access(s, ptr, &s->stageData.monitor_markWord, 8, false))
                    return;

                s->stageData.age = (s->stageData.monitor_markWord >> 3) & 0x1111;
            }
            else
                s->stageData.age = (s->stageData.common_m_value >> 3) & 0x1111;

            if (s->stageData.age < s->stageData.pars.ageThreshold)
            {
                s->stageData.dest_attr = s->stageData.src_region_attr;
                s->stageData.dest_attr_ptr = s->stageData.src_region_attr_ptr;
            }
        }

        s->stageData.plab_idx = (int8_t)(s->stageData.dest_attr >> 8);
        s->sub_stage = 5;

        break;
    
    case 5:
    {
        int8_t dest_attr_type = (int8_t)(d->dest_attr >> 8);
        if (!hwgc_access(s, d->pars.plabAllocatorPtr + 0x10 + dest_attr_type * 8, &d->buffer_temp, 8, false))
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
            uint64_t writeValue = d->region_top + d->size * 8;
            if (!hwgc_access(s, d->buffer + 0x30, &d->region_top, 8, true))
                return;
            d->region_top = writeValue;
            s->sub_stage = 12;
        }
        else
        {
            d->to_obj = 0;
            hwgc_goto_stage(s, STAGE_ALLOC, 0);
        }
        break;

    case 12:
        d->writeSrcMW = (d->to_obj & ~0x3) | 0x3;

        if (!hwgc_access(s, d->from_obj, &d->writeSrcMW, 8, true))
            return;

        s->sub_stage = 13;
        break;

    case 13:
        if (!hwgc_access(s, d->from_region + 256, &d->young_index, 4, false))
            return;

        s->sub_stage = 14;
        break;

    case 14:
        if (!hwgc_access(s,
                         d->pars.youngWordsBase + d->young_index * 8,
                         &d->originValue,
                         8,
                         false))
            return;

        s->sub_stage = 15;
        break;

    case 15:
    {
        size_t temp = d->originValue + d->size;

        if (!hwgc_access(s,
                         d->pars.youngWordsBase + d->young_index * 8,
                         &temp,
                         8,
                         true))
            return;

        s->sub_stage = 16;
        break;
    }

    case 16:
        d->new_mark = d->common_m_value;

        if ((int8_t)(d->dest_attr >> 8) == 0 &&
            (d->common_m_value & 0x1) != 0)
        {
            d->new_mark = (d->common_m_value & ~(0x1111 << 3)) |
                          ((((d->age + 1) < 15 ? d->age + 1 : d->age) &
                            0x1111) << 3);
        }

        if (!hwgc_access(s, d->to_obj, &d->new_mark, 8, true))
            return;

        s->sub_stage = 17;
        break;

    case 17:
        if ((int8_t)(d->dest_attr >> 8) == 0 &&
            (d->common_m_value & 0x1) == 0)
        {
            uintptr_t ptr = (d->common_m_value & 0x2)
                              ? (d->common_m_value ^ 0x2)
                              : d->common_m_value;

            if (!hwgc_access(s, ptr, &d->originValue, 8, false))
                return;

            s->sub_stage = 18;
        }
        else
        {
            s->sub_stage = 19;
        }
        break;

    case 18:
    {
        uintptr_t ptr = (d->common_m_value & 0x2)
                          ? (d->common_m_value ^ 0x2)
                          : d->common_m_value;

        d->originValue = (d->originValue & ~(0x1111 << 3)) |
                         ((((d->age + 1) < 15 ? d->age + 1 : d->age) &
                           0x1111) << 3);

        if (!hwgc_access(s, ptr, &d->originValue, 8, true))
            return;

        s->sub_stage = 19;
        break;
    }

    case 19:
        d->i = 1;
        hwgc_goto_stage(s, STAGE_COPY, 0);
        break;

    case 20:
        if (d->kid == 4)
        {
            hwgc_goto_stage(s, STAGE_COMMON_OOP, 4);
            return;
        }

        hwgc_goto_stage(s, STAGE_TRACE, 0);
        break;

    default:
        hwgc_goto_stage(s, STAGE_COPY2SURVIVOR, 0);
        break;
    }
}

static void stage_alloc_function(HWGCDevState *s)
{
    HWGCStageData *d = &s->stageData;

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
            s->sub_stage = 7;
        }
        break;

    case 2:
        if (!hwgc_access(s, d->buffer_temp, &d->buffer, 8, false))
            return;

        if (!hwgc_access(s, d->buffer + 0x30, &d->region_top, 8, false))
            return;

        if (!hwgc_access(s, d->buffer + 0x38, &d->region_end, 8, false))
            return;

        s->sub_stage = 5;
        break;

    case 5:
        d->dest_attr_type = 1;
        d->dest_attr = (d->dest_attr & 0xff) |
                       ((uint16_t)((int16_t)d->dest_attr_type) << 8);

        if ((d->region_end - d->region_top) / 8 >= d->size)
        {
            d->to_obj = d->region_top;
            d->region_top = d->region_top + d->size * 8;

            if (!hwgc_access(s, d->buffer + 0x30, &d->region_top, 8, true))
                return;

            s->sub_stage = 6;
        }
        else
        {
            d->previous = STAGE_ALLOC;
            d->previous_sub_stage = 6;

            hwgc_goto_stage(s, STAGE_ALLOCATE_DIRECT, 0);
        }
        break;

    case 6:
        if (!hwgc_access(s, d->dest_attr_ptr + 1, &d->dest_attr_type, 1, true))
            return;

        s->sub_stage = 7;
        break;

    case 7:
        hwgc_goto_stage(s, STAGE_COPY2SURVIVOR, 12);
        break;

    default:
        hwgc_goto_stage(s, STAGE_ALLOC, 0);
        break;
    }
}

static void stage_allocate_direct_function(HWGCDevState *s)
{
    HWGCStageData *d = &s->stageData;

    switch (s->sub_stage)
    {
    case 0:
        if (d->dest_attr_type == 0)
            d->plab_stats_ptr = d->pars.g1h + 0x250;
        else if (d->dest_attr_type == 1)
            d->plab_stats_ptr = d->pars.g1h + 0x2e0;
        else
            d->plab_stats_ptr = d->pars.g1h + 0x2e0;

        if (!hwgc_access(s, d->plab_stats_ptr + 0x30, &d->originValue, 8, false))
            return;

        s->sub_stage = 1;
        break;

    case 1:
        d->plab_word_size = MIN(MAX(d->originValue, 0x102), 0x40000);
        d->required_in_plab = d->size + 0x2;

        if (!hwgc_access(s, d->pars.plabAllocatorPtr + 0x8, &d->allocator_ptr, 8, false))
            return;

        s->sub_stage = 2;
        break;

    case 2:
    {
        bool may_throw_away_buffer =
            d->required_in_plab * 100 < d->plab_word_size * 0xa;

        if ((d->required_in_plab <= d->plab_word_size) && may_throw_away_buffer)
        {
            if (!hwgc_access(s,
                             d->pars.plabAllocatorPtr + 0x10 + d->dest_attr_type * 8,
                             &d->buffer_temp,
                             8,
                             false))
                return;

            s->sub_stage = 3;
        }
        else
        {
            s->sub_stage = 25;
        }
        break;
    }

    case 3:
        if (!hwgc_access(s, d->buffer_temp, &d->buffer, 8, false))
            return;

        if (!hwgc_access(s, d->buffer + 0x30, &d->region_top, 8, false))
            return;

        if (!hwgc_access(s, d->buffer + 0x40, &d->region_end, 8, false))
            return;

        s->sub_stage = 6;
        break;

    case 6:
        if (d->region_top < d->region_end)
        {
            size_t words = (d->region_end - d->region_top) / 8;

            if (words >= (d->pars.useCompressedKlassPointers ? 2 : 3))
            {
                size_t payload_size = words -
                                      (d->pars.useCompressedKlassPointers ? 2 : 3);
                uint32_t len = (uint32_t)(payload_size * 8 / 4);

                d->alloc_klass_ptr = d->pars.intArrayKlassObj;

                if (!hwgc_access(s,
                                 d->region_top +
                                     (d->pars.useCompressedKlassPointers ? 12 : 16),
                                 &len,
                                 4,
                                 true))
                    return;

                s->sub_stage = 7;
            }
            else if (words > 0)
            {
                d->alloc_klass_ptr = d->pars.objectKlass;
                s->sub_stage = 7;
            }
            else
            {
                s->sub_stage = 14;
            }
        }
        else
        {
            s->sub_stage = 14;
        }
        break;

    case 7:
        d->originValue = 1;

        if (!hwgc_access(s, d->region_top, &d->originValue, 8, true))
            return;

        s->sub_stage = 8;
        break;

    case 8:
        if (d->pars.useCompressedKlassPointers)
        {
            d->alloc_klass_ptr =
                (d->alloc_klass_ptr - d->pars.compressedKlassPointerBase) >>
                d->pars.compressedKlassPointerShift;

            uint32_t narrow_klass = (uint32_t)d->alloc_klass_ptr;

            if (!hwgc_access(s, d->region_top + 0x8, &narrow_klass, 4, true))
                return;
        }
        else
        {
            if (!hwgc_access(s, d->region_top + 0x8, &d->alloc_klass_ptr, 8, true))
                return;
        }

        s->sub_stage = 9;
        break;

    case 9:
        if (!hwgc_access(s, d->buffer + 0x38, &d->region_end, 8, true))
            return;

        if (!hwgc_access(s, d->buffer + 0x30, &d->region_end, 8, true))
            return;

        if (!hwgc_access(s, d->buffer + 0x28, &d->region_end, 8, true))
            return;

        s->sub_stage = 12;
        break;

    case 12:
        if (!hwgc_access(s, d->buffer + 0x50, &d->originValue, 8, false))
            return;

        s->sub_stage = 13;
        break;

    case 13:
        d->originValue = d->originValue + (d->region_end - d->region_top) / 8;

        if (!hwgc_access(s, d->buffer + 0x50, &d->originValue, 8, true))
            return;

        s->sub_stage = 14;
        break;

    case 14:
        if (!hwgc_access(s,
                         d->pars.plabAllocatorPtr + 0x30 + d->dest_attr_type * 8,
                         &d->originValue,
                         8,
                         false))
            return;

        s->sub_stage = 15;
        break;

    case 15:
        d->originValue = d->originValue + 1;

        if (!hwgc_access(s,
                         d->pars.plabAllocatorPtr + 0x30 + d->dest_attr_type * 8,
                         &d->originValue,
                         8,
                         true))
            return;

        s->sub_stage = 16;
        break;

    case 16:
        d->min_word_size = d->required_in_plab;
        d->desired_word_size = d->plab_word_size;
        d->during_gc_select = 0;

        hwgc_goto_stage(s, STAGE_ALLOCATE_DURING_GC, 0);
        break;

    case 17:
        if (d->to_obj != 0)
        {
            if (!hwgc_access(s, d->buffer + 0x20, &d->actual_plab_size, 8, true))
                return;

            s->sub_stage = 18;
        }
        else
        {
            s->sub_stage = 25;
        }
        break;

    case 18:
        if (!hwgc_access(s, d->buffer + 0x28, &d->to_obj, 8, true))
            return;

        s->sub_stage = 19;
        break;

    case 19:
        d->originValue = d->to_obj + d->actual_plab_size * 8;

        if (!hwgc_access(s, d->buffer + 0x40, &d->originValue, 8, true))
            return;

        s->sub_stage = 20;
        break;

    case 20:
        d->originValue = d->to_obj + (d->actual_plab_size - 2) * 8;

        if (!hwgc_access(s, d->buffer + 0x38, &d->originValue, 8, true))
            return;

        s->sub_stage = 21;
        break;

    case 21:
        if (!hwgc_access(s, d->buffer + 0x48, &d->originValue, 8, false))
            return;

        s->sub_stage = 22;
        break;

    case 22:
        d->originValue = d->originValue + d->actual_plab_size;

        if (!hwgc_access(s, d->buffer + 0x48, &d->originValue, 8, true))
            return;

        s->sub_stage = 23;
        break;

    case 23:
    {
        size_t delta = d->actual_plab_size - 0x2;

        if (delta >= d->size)
        {
            d->originValue = d->to_obj + d->size * 8;
        }
        else
        {
            d->originValue = d->to_obj;
            d->to_obj = 0;
        }

        if (!hwgc_access(s, d->buffer + 0x30, &d->originValue, 8, true))
            return;

        s->sub_stage = 24;
        break;
    }

    case 24:
        hwgc_return_previous(s);
        return;

    case 25:
        d->min_word_size = d->size;
        d->desired_word_size = d->size;
        d->during_gc_select = 1;

        hwgc_goto_stage(s, STAGE_ALLOCATE_DURING_GC, 0);
        break;

    default:
        hwgc_goto_stage(s, STAGE_ALLOCATE_DIRECT, 0);
        break;
    }
}

static void stage_allocate_during_gc_function(HWGCDevState *s)
{
    HWGCStageData *d = &s->stageData;

    switch (s->sub_stage)
    {
    case 0:
        if (d->dest_attr_type == 0)
        {
            if (!hwgc_access(s, d->allocator_ptr + 0x28, &d->region_ptr, 8, false))
                return;
        }
        else
        {
            d->region_ptr = d->allocator_ptr + 0x30;
        }

        s->sub_stage = 1;
        break;

    case 1:
        if (!hwgc_access(s, d->region_ptr + 0x8, &d->alloc_region, 8, false))
            return;

        s->sub_stage = 2;
        break;

    case 2:
        if (d->dest_attr_type == 0)
        {
            d->par_alloc_iml_sel = 0;
            hwgc_goto_stage(s, STAGE_PAR_ALLOCATE_IML, 0);
        }
        else
        {
            d->par_alloc_sel = 0;
            d->bot_updates = true;
            hwgc_goto_stage(s, STAGE_PAR_ALLOCATE, 0);
        }
        break;

    case 3:
        if (d->to_obj == 0)
        {
            if (!hwgc_access(s, d->allocator_ptr + 0x10, &d->originValue, 1, false))
                return;

            s->sub_stage = 5;
        }
        else
        {
            s->sub_stage = 4;
        }
        break;

    case 4:
        hwgc_goto_stage(s,
                        STAGE_ALLOCATE_DIRECT,
                        d->during_gc_select ? 24 : 17);
        return;

    case 5:
    {
        bool is_full = d->dest_attr_type == 0
                         ? (d->originValue & 0x1) != 0
                         : (d->originValue & 0x2) != 0;

        if (!is_full)
        {
            if (!hwgc_access(s, d->pars.lockPtr, &d->pars.thread, 8, true))
                return;

            s->sub_stage = 6;
        }
        else
        {
            s->sub_stage = 4;
        }
        break;
    }

    case 6:
        if (d->dest_attr_type == 0)
        {
            d->par_alloc_iml_sel = 1;
            hwgc_goto_stage(s, STAGE_PAR_ALLOCATE_IML, 0);
        }
        else
        {
            d->par_alloc_sel = 1;
            d->bot_updates = true;
            hwgc_goto_stage(s, STAGE_PAR_ALLOCATE, 0);
        }
        break;

    case 7:
        if (d->to_obj == 0)
        {
            hwgc_goto_stage(s, STAGE_ATTEMPT_ALLOC, 0);
        }
        else
        {
            s->sub_stage = 8;
        }
        break;

    case 8:
        if (d->to_obj == 0)
        {
            uintptr_t addr = d->dest_attr_type == 0
                               ? d->allocator_ptr + 0x10
                               : d->allocator_ptr + 0x11;
            uint8_t full = 1;

            if (!hwgc_access(s, addr, &full, 1, true))
                return;
        }

        s->sub_stage = 9;
        break;

    case 9:
        d->originValue = 0;

        if (!hwgc_access(s, d->pars.lockPtr, &d->originValue, 8, true))
            return;

        s->sub_stage = 4;
        break;

    default:
        hwgc_goto_stage(s, STAGE_ALLOCATE_DURING_GC, 0);
        break;
    }
}

static void stage_par_allocate_iml_function(HWGCDevState *s)
{
    HWGCStageData *d = &s->stageData;

    switch (s->sub_stage)
    {
    case 0:
        if (!hwgc_access(s, d->alloc_region + 0x10, &d->alloc_top, 8, false))
            return;

        if (!hwgc_access(s, d->alloc_region + 0x8, &d->alloc_end, 8, false))
            return;

        s->sub_stage = 2;
        break;

    case 2:
    {
        size_t available = (d->alloc_end - d->alloc_top) / 8;

        d->want_to_allocate =
            available > d->desired_word_size ? d->desired_word_size : available;

        if (d->want_to_allocate >= d->min_word_size)
        {
            d->actual_plab_size = d->want_to_allocate;
            d->originValue = d->alloc_top + d->want_to_allocate * 8;
            d->to_obj = d->alloc_top;

            if (!hwgc_access(s, d->alloc_region + 0x10, &d->originValue, 8, true))
                return;
        }
        else
        {
            d->to_obj = 0;
        }

        s->sub_stage = 3;
        break;
    }

    case 3:
        if (d->par_alloc_iml_sel == 2)
            hwgc_goto_stage(s, STAGE_PAR_ALLOCATE, 1);
        else if (d->par_alloc_iml_sel == 1)
            hwgc_goto_stage(s, STAGE_ALLOCATE_DURING_GC, 7);
        else
            hwgc_goto_stage(s, STAGE_ALLOCATE_DURING_GC, 3);
        break;

    default:
        hwgc_goto_stage(s, STAGE_PAR_ALLOCATE_IML, 0);
        break;
    }
}

static void stage_par_allocate_function(HWGCDevState *s)
{
    HWGCStageData *d = &s->stageData;

    switch (s->sub_stage)
    {
    case 0:
        d->par_alloc_iml_sel = 2;
        hwgc_goto_stage(s, STAGE_PAR_ALLOCATE_IML, 0);
        return;

    case 1:
        if (d->to_obj != 0 && d->bot_updates)
        {
            d->blk_start = d->to_obj;
            d->blk_end = d->to_obj + d->actual_plab_size * 8;
            d->bot_part_ptr = d->alloc_region + 0x20;

            if (!hwgc_access(s,
                             d->bot_part_ptr,
                             &d->next_offset_threshold,
                             8,
                             false))
                return;

            s->sub_stage = 2;
        }
        else
        {
            s->sub_stage = 13;
        }
        break;

    case 2:
        if (d->blk_end > d->next_offset_threshold)
        {
            if (!hwgc_access(s, d->bot_part_ptr + 0x8, &d->index, 8, false))
                return;

            s->sub_stage = 3;
        }
        else
        {
            s->sub_stage = 13;
        }
        break;

    case 3:
        if (!hwgc_access(s, d->bot_part_ptr + 0x10, &d->bot_ptr, 8, false))
            return;

        s->sub_stage = 4;
        break;

    case 4:
        if (!hwgc_access(s, d->bot_ptr + 0x10, &d->array, 8, false))
            return;

        s->sub_stage = 5;
        break;

    case 5:
    {
        uint8_t value = (uint8_t)((d->next_offset_threshold - d->blk_start) / 8);

        if (!hwgc_access(s, d->array + d->index, &value, 1, true))
            return;

        s->sub_stage = 6;
        break;
    }

    case 6:
        if (!hwgc_access(s, d->bot_ptr, &d->reserved_start, 8, false))
            return;

        s->sub_stage = 7;
        break;

    case 7:
    {
        size_t end_index = (d->blk_end - 8 - d->reserved_start) >> 9;
        uintptr_t rem_st = d->reserved_start + ((d->index + 1) << 6) * 8;
        uintptr_t rem_end = d->reserved_start + ((end_index << 6) + 64) * 8;

        d->start_card = (rem_st - d->reserved_start) >> 9;
        d->end_card = (rem_end - 8 - d->reserved_start) >> 9;

        if (d->index + 1 <= end_index &&
            rem_st < rem_end &&
            d->start_card <= d->end_card)
        {
            d->start_card_for_region = d->start_card;
            d->ct_offset = 0xff;
            d->i = 0;
            s->sub_stage = 8;
        }
        else
        {
            s->sub_stage = 11;
        }
        break;
    }

    case 8:
        if (d->i < 14)
        {
            d->reach = d->start_card - 1 +
                       ((1u << (4 * (d->i + 1))) - 1);
            d->ct_offset = 64 + d->i;
            d->num_cards =
                (d->reach >= d->end_card ? d->end_card : d->reach) -
                d->start_card_for_region + 1;
            d->begin = d->array + d->start_card_for_region;

            d->i++;
            s->sub_stage = 9;
        }
        else
        {
            s->sub_stage = 11;
        }
        break;

    case 9:
        if (d->num_cards > 0)
        {
            uintptr_t addr = d->begin;

            d->begin++;
            d->num_cards--;

            if (!hwgc_access(s, addr, &d->ct_offset, 1, true))
                return;
        }
        else
        {
            s->sub_stage = 10;
        }
        break;

    case 10:
        d->start_card_for_region = d->reach + 1;

        if (d->reach >= d->end_card)
            s->sub_stage = 11;
        else
            s->sub_stage = 8;
        break;

    case 11:
    {
        size_t end_index = (d->blk_end - 8 - d->reserved_start) >> 9;

        d->index = end_index + 1;
        d->next_offset_threshold =
            d->reserved_start + ((end_index << 6) + 64) * 8;

        if (!hwgc_access(s,
                         d->bot_part_ptr,
                         &d->next_offset_threshold,
                         8,
                         true))
            return;

        s->sub_stage = 12;
        break;
    }

    case 12:
        if (!hwgc_access(s, d->bot_part_ptr + 0x8, &d->index, 8, true))
            return;

        s->sub_stage = 13;
        break;

    case 13:
        if (d->par_alloc_sel == 2)
            hwgc_goto_stage(s, STAGE_ATTEMPT_ALLOC, 17);
        else if (d->par_alloc_sel == 1)
            hwgc_goto_stage(s, STAGE_ALLOCATE_DURING_GC, 7);
        else
            hwgc_goto_stage(s, STAGE_ALLOCATE_DURING_GC, 3);
        break;

    default:
        hwgc_goto_stage(s, STAGE_PAR_ALLOCATE, 0);
        break;
    }
}

static void stage_attempt_alloc_function(HWGCDevState *s)
{
    HWGCStageData *d = &s->stageData;

    switch (s->sub_stage)
    {
    case 0:
        if (d->alloc_region != d->pars.dummyRegion)
        {
            if (!hwgc_access(s, d->alloc_region, &d->alloc_end, 8, false))
                return;

            s->sub_stage = 1;
        }
        else
        {
            s->sub_stage = 10;
        }
        break;

    case 1:
        if (!hwgc_access(s, d->alloc_region + 0x10, &d->alloc_top, 8, false))
            return;

        s->sub_stage = 2;
        break;

    case 2:
        if (!hwgc_access(s, d->region_ptr + 0x18, &d->originValue, 8, false))
            return;

        s->sub_stage = 3;
        break;

    case 3:
        d->allocated_bytes = d->alloc_top - d->alloc_end - d->originValue;

        if (!hwgc_access(s, d->pars.g1h + 0x240, &d->originValue, 8, false))
            return;

        s->sub_stage = 4;
        break;

    case 4:
        d->originValue = d->originValue + d->allocated_bytes;

        if (!hwgc_access(s, d->pars.g1h + 0x240, &d->originValue, 8, true))
            return;

        s->sub_stage = 5;
        break;

    case 5:
        if (!hwgc_access(s, d->region_ptr + 0x40, &d->type, 1, false))
            return;

        s->sub_stage = 6;
        break;

    case 6:
    {
        uintptr_t ptr = d->pars.g1h + (d->type == 1 ? 0xa0 : 0x3f8) + 0x10;

        if (!hwgc_access(s, ptr, &d->originValue, 8, false))
            return;

        s->sub_stage = 7;
        break;
    }

    case 7:
    {
        uintptr_t ptr = d->pars.g1h + (d->type == 1 ? 0xa0 : 0x3f8) + 0x10;

        d->originValue = d->originValue +
                         (d->type == 1 ? 1 : d->allocated_bytes);

        if (!hwgc_access(s, ptr, &d->originValue, 8, true))
            return;

        s->sub_stage = 21;
        break;
    }

    case 21:
        if (!hwgc_access(s, d->pars.g1h + 0x3c1, &d->originValue, 1, false))
            return;

        s->sub_stage = 22;
        break;

    case 22:
        if ((d->originValue & 0xff) && d->allocated_bytes > 0)
        {
            if (!hwgc_access(s, d->pars.g1h + 0x4e8, &d->cm, 8, false))
                return;

            s->sub_stage = 23;
        }
        else
        {
            s->sub_stage = 8;
        }
        break;

    case 23:
        if (!hwgc_access(s, d->alloc_region + 0xe8, &d->next_top, 8, false))
            return;

        s->sub_stage = 24;
        break;

    case 24:
        if (!hwgc_access(s, d->cm + 0xb0, &d->root_regions_array, 8, false))
            return;

        if (!hwgc_access(s, d->cm + 0xb0 + 0x10, &d->originValue, 8, false))
            return;

        s->sub_stage = 26;
        break;

    case 26:
        d->mem_region = d->root_regions_array + d->originValue * 0x10;

        if (!hwgc_access(s, d->mem_region, &d->next_top, 8, true))
            return;

        s->sub_stage = 27;
        break;

    case 27:
        d->originValue = d->originValue + 1;

        if (!hwgc_access(s, d->cm + 0xb0 + 0x10, &d->originValue, 8, true))
            return;

        s->sub_stage = 28;
        break;

    case 28:
        d->originValue = (d->alloc_top - d->next_top) / 8;

        if (!hwgc_access(s, d->mem_region + 0x8, &d->originValue, 8, true))
            return;

        s->sub_stage = 8;
        break;

    case 8:
        d->originValue = 0;

        if (!hwgc_access(s, d->region_ptr + 0x18, &d->originValue, 8, true))
            return;

        s->sub_stage = 9;
        break;

    case 9:
        if (!hwgc_access(s, d->region_ptr + 0x8, &d->pars.dummyRegion, 8, true))
            return;

        s->sub_stage = 10;
        break;

    case 10:
        hwgc_goto_stage(s, STAGE_NEW_GC_ALLOC, 0);
        break;

    case 11:
        if (d->new_alloc_region != 0)
        {
            d->originValue = 0;

            if (!hwgc_access(s, d->new_alloc_region + 0xa8, &d->originValue, 8, true))
                return;

            s->sub_stage = 12;
        }
        else
        {
            s->sub_stage = 20;
        }
        break;

    case 12:
        if (!hwgc_access(s, d->new_alloc_region, &d->alloc_end, 8, false))
            return;

        if (!hwgc_access(s, d->new_alloc_region + 0x10, &d->alloc_top, 8, false))
            return;

        s->sub_stage = 14;
        break;

    case 14:
        d->originValue = d->alloc_top - d->alloc_end;

        if (!hwgc_access(s, d->region_ptr + 0x18, &d->originValue, 8, true))
            return;

        s->sub_stage = 15;
        break;

    case 15:
        if (!hwgc_access(s, d->region_ptr + 0x20, &d->bot_updates, 1, false))
            return;

        s->sub_stage = 16;
        break;

    case 16:
        d->alloc_region = d->new_alloc_region;
        d->min_word_size = d->desired_word_size;
        d->par_alloc_sel = 2;

        hwgc_goto_stage(s, STAGE_PAR_ALLOCATE, 0);
        return;

    case 17:
        if (!hwgc_access(s, d->region_ptr + 0x8, &d->new_alloc_region, 8, true))
            return;

        s->sub_stage = 18;
        break;

    case 18:
        if (!hwgc_access(s, d->region_ptr + 0x10, &d->originValue, 4, false))
            return;

        s->sub_stage = 19;
        break;

    case 19:
    {
        uint32_t count = (uint32_t)d->originValue + 1;

        if (!hwgc_access(s, d->region_ptr + 0x10, &count, 4, true))
            return;

        s->sub_stage = 20;
        break;
    }

    case 20:
        if (d->new_alloc_region != 0 && d->to_obj != 0)
            d->actual_plab_size = d->desired_word_size;
        else
            d->to_obj = 0;

        hwgc_goto_stage(s, STAGE_ALLOCATE_DURING_GC, 8);
        return;

    default:
        hwgc_goto_stage(s, STAGE_ATTEMPT_ALLOC, 0);
        break;
    }
}

static void stage_new_gc_alloc_function(HWGCDevState *s)
{
    HWGCStageData *d = &s->stageData;

    switch (s->sub_stage)
    {
    case 0:
        if (!hwgc_access(s, d->region_ptr + 0x30, &d->region_node_index, 4, false))
            return;

        s->sub_stage = 1;
        break;

    case 1:
        if (!hwgc_access(s,
                         d->pars.g1h + 0x3f8 + 0x8,
                         &d->grow_array_ptr,
                         8,
                         false))
            return;

        s->sub_stage = 2;
        break;

    case 2:
        if (!hwgc_access(s, d->pars.g1h + 0x430, &d->policy_ptr, 8, false))
            return;

        s->sub_stage = 3;
        break;

    case 3:
        d->heap_region_type = d->type == 1 ? 0x10 : 0x3;
        d->allocate_free_sel = false;

        hwgc_goto_stage(s, STAGE_ALLOC_FREE_REGION, 0);
        break;

    case 16:
        if (!hwgc_access(s, d->pars.g1h + 0x370, &d->expand_failure, 1, false))
            return;

        s->sub_stage = 17;
        break;

    case 17:
        if (d->new_alloc_region == 0 && (d->expand_failure & 0xff))
        {
            ++alloc_irq;

            d->softPars.par0 = d->region_node_index;

            s->wake_stage = STAGE_NEW_GC_ALLOC;
            s->wake_sub_stage = 18;

            hwgc_goto_stage(s, STAGE_DEBUG, 0);

            if (qatomic_read(&s->status) & HWGC_STATUS_IRQ)
            {
                bql_lock();
                hwgc_raise_irq(s, DEBUG_IRQ);
                bql_unlock();
            }

            return;
        }

        s->sub_stage = 4;
        break;

    case 18:
    {
        bool tag = d->softPars.res & 0xff;
        uint8_t zero = 0;

        if (tag)
        {
            d->allocate_free_sel = true;
            hwgc_goto_stage(s, STAGE_ALLOC_FREE_REGION, 0);
        }
        else
        {
            if (!hwgc_access(s, d->pars.g1h + 0x370, &zero, 1, true))
                return;

            s->sub_stage = 4;
        }
        break;
    }

    case 4:
        if (d->new_alloc_region != 0)
        {
            if (!hwgc_access(s,
                             d->new_alloc_region + 0xbc,
                             &d->heap_region_type,
                             4,
                             true))
                return;

            s->sub_stage = 5;
        }
        else
        {
            s->sub_stage = 15;
        }
        break;

    case 5:
        if (d->heap_region_type == 0x3)
        {
            if (!hwgc_access(s, d->grow_array_ptr, &d->originValue, 8, false))
                return;

            s->sub_stage = 6;
        }
        else
        {
            s->sub_stage = 10;
        }
        break;

    case 6:
        d->array_max = d->originValue >> 32;
        d->array_len = (uint32_t)d->originValue;

        if (d->array_len == d->array_max)
        {
            ++grow_irq;

            d->softPars.par0 = d->grow_array_ptr;
            d->softPars.par1 = d->array_len;

            s->wake_stage = STAGE_NEW_GC_ALLOC;
            s->wake_sub_stage = 7;

            hwgc_goto_stage(s, STAGE_DEBUG, 0);

            if (qatomic_read(&s->status) & HWGC_STATUS_IRQ)
            {
                bql_lock();
                hwgc_raise_irq(s, ALLOC_SLOW_IRQ);
                bql_unlock();
            }

            return;
        }

        s->sub_stage = 7;
        break;

    case 7:
    {
        uint32_t new_len = d->array_len + 1;

        if (!hwgc_access(s, d->grow_array_ptr, &new_len, 4, true))
            return;

        s->sub_stage = 8;
        break;
    }

    case 8:
        if (!hwgc_access(s, d->grow_array_ptr + 0x8, &d->data_ptr, 8, false))
            return;

        s->sub_stage = 9;
        break;

    case 9:
        if (!hwgc_access(s,
                         d->data_ptr + d->array_len * 8,
                         &d->new_alloc_region,
                         8,
                         true))
            return;

        s->sub_stage = 10;
        break;

    case 10:
        if (!hwgc_access(s, d->new_alloc_region + 0xb0, &d->count_per_node, 8, false))
            return;

        if (!hwgc_access(s, d->new_alloc_region + 0xb8, &d->originValue, 8, false))
            return;

        s->sub_stage = 12;
        break;

    case 12:
    {
        uint32_t state_value;

        d->array_len = d->originValue >> 32;
        d->array_max = (uint32_t)d->originValue;

        state_value = (d->array_len & 0x2) != 0
                        ? 2
                        : ((d->array_len & 0x10) != 0 ? 0 : 1);

        if (state_value != 1)
        {
            if (!hwgc_access(s, d->count_per_node + 0xf0, &state_value, 4, true))
                return;
        }

        s->sub_stage = 13;
        break;
    }

    case 13:
        if (!hwgc_access(s,
                         d->pars.g1h + 0x580 + 0x10,
                         &d->count_per_node,
                         8,
                         false))
            return;

        s->sub_stage = 14;
        break;

    case 14:
    {
        bool needs_remset_update = (d->array_len & 0x10) == 0;

        if (!hwgc_access(s,
                         d->count_per_node + d->array_max * 2,
                         &needs_remset_update,
                         1,
                         true))
            return;

        s->sub_stage = 15;
        break;
    }

    case 15:
        hwgc_goto_stage(s, STAGE_ATTEMPT_ALLOC, 11);
        return;

    default:
        hwgc_goto_stage(s, STAGE_NEW_GC_ALLOC, 0);
        break;
    }
}

static void stage_alloc_free_region_function(HWGCDevState *s)
{
    HWGCStageData *d = &s->stageData;

    switch (s->sub_stage)
    {
    case 0:
        if (!hwgc_access(s, d->pars.numaPtr + 0x18, &d->active_node_ids, 4, false))
            return;

        s->sub_stage = 1;
        break;

    case 1:
        d->new_alloc_region = 0;
        d->free_list_ptr = d->pars.g1h + 0x130 + 0xb0;
        d->from_head = (d->heap_region_type & 0x2) == 0;

        if (d->region_node_index != UINT_MAX - 1 && d->active_node_ids > 1)
        {
            if (!hwgc_access(s, d->pars.numaPtr + 0x20, &d->region_size, 4, false))
                return;

            s->sub_stage = 2;
        }
        else
        {
            s->sub_stage = 12;
        }
        break;

    case 2:
        if (!hwgc_access(s, d->pars.numaPtr + 0x28, &d->page_size, 4, false))
            return;

        s->sub_stage = 3;
        break;

    case 3:
    {
        uintptr_t addr;

        d->cur_depth = 0;
        d->max_depth =
            3 * MAX((uint32_t)(d->page_size / d->region_size), 1u) *
            d->active_node_ids;

        addr = d->free_list_ptr + (d->from_head ? 0x28 : 0x30);

        if (!hwgc_access(s, addr, &d->cur, 8, false))
            return;

        s->sub_stage = 4;
        break;
    }

    case 4:
        if (d->cur != 0 && d->cur_depth < d->max_depth)
        {
            if (!hwgc_access(s, d->cur + 0x120, &d->originValue, 4, false))
                return;

            s->sub_stage = 5;
        }
        else
        {
            s->sub_stage = 6;
        }
        break;

    case 5:
        if (d->region_node_index == (uint32_t)d->originValue)
        {
            s->sub_stage = 6;
        }
        else
        {
            uintptr_t addr;

            d->cur_depth++;
            addr = d->cur + (d->from_head ? 0xd0 : 0xd8);

            if (!hwgc_access(s, addr, &d->cur, 8, false))
                return;

            s->sub_stage = 4;
        }
        break;

    case 6:
        if (d->cur == 0 || d->cur_depth >= d->max_depth)
        {
            d->new_alloc_region = 0;
            s->sub_stage = 12;
        }
        else
        {
            d->new_alloc_region = d->cur;

            if (!hwgc_access(s, d->new_alloc_region + 0xd8, &d->prev, 8, false))
                return;

            s->sub_stage = 7;
        }
        break;

    case 7:
        if (!hwgc_access(s, d->new_alloc_region + 0xd0, &d->next, 8, false))
            return;

        s->sub_stage = 8;
        break;

    case 8:
    {
        uintptr_t addr = d->prev == 0
                           ? d->free_list_ptr + 0x28
                           : d->prev + 0xd0;

        if (!hwgc_access(s, addr, &d->next, 8, true))
            return;

        s->sub_stage = 9;
        break;
    }

    case 9:
    {
        /*
         * 原代码这里是:
         *     next == 0 ? free_list_ptr + 0x30 : prev + 0xd8
         *
         * 如果这里确认为双向链表删除逻辑，通常应是:
         *     next == 0 ? free_list_ptr + 0x30 : next + 0xd8
         *
         * 这里先保持原逻辑不改。
         */
        uintptr_t addr = d->next == 0
                           ? d->free_list_ptr + 0x30
                           : d->prev + 0xd8;

        if (!hwgc_access(s, addr, &d->prev, 8, true))
            return;

        s->sub_stage = 10;
        break;
    }

    case 10:
        d->originValue = 0;

        if (!hwgc_access(s, d->new_alloc_region + 0xd0, &d->originValue, 8, true))
            return;

        s->sub_stage = 11;
        break;

    case 11:
        if (!hwgc_access(s, d->new_alloc_region + 0xd8, &d->originValue, 8, true))
            return;

        s->sub_stage = 12;
        break;

    case 12:
        if (!hwgc_access(s, d->free_list_ptr + 0x10, &d->active_node_ids, 4, false))
            return;

        s->sub_stage = 13;
        break;

    case 13:
        if (d->new_alloc_region == 0)
        {
            if (d->active_node_ids == 0)
            {
                d->new_alloc_region = 0;
                s->sub_stage = 18;
            }
            else
            {
                uintptr_t addr = d->free_list_ptr + (d->from_head ? 0x28 : 0x30);

                if (!hwgc_access(s, addr, &d->new_alloc_region, 8, false))
                    return;

                s->sub_stage = 14;
            }
        }
        else
        {
            s->sub_stage = 18;
        }
        break;

    case 14:
    {
        uintptr_t addr = d->new_alloc_region + (d->from_head ? 0xd0 : 0xd8);

        if (!hwgc_access(s, addr, &d->originValue, 8, false))
            return;

        s->sub_stage = 15;
        break;
    }

    case 15:
    {
        uintptr_t addr = d->free_list_ptr + (d->from_head ? 0x28 : 0x30);

        if (!hwgc_access(s, addr, &d->originValue, 8, true))
            return;

        s->sub_stage = 16;
        break;
    }

    case 16:
    {
        uintptr_t addr;

        if (d->from_head)
            addr = d->originValue == 0 ? d->free_list_ptr + 0x30
                                       : d->originValue + 0xd8;
        else
            addr = d->originValue == 0 ? d->free_list_ptr + 0x28
                                       : d->originValue + 0xd0;

        d->originValue = 0;

        if (!hwgc_access(s, addr, &d->originValue, 8, true))
            return;

        s->sub_stage = 17;
        break;
    }

    case 17:
    {
        uintptr_t addr = d->new_alloc_region + (d->from_head ? 0xd0 : 0xd8);

        if (!hwgc_access(s, addr, &d->originValue, 8, true))
            return;

        s->sub_stage = 18;
        break;
    }

    case 18:
        if (d->new_alloc_region != 0)
        {
            if (!hwgc_access(s, d->free_list_ptr + 0x38, &d->originValue, 8, false))
                return;

            s->sub_stage = 19;
        }
        else
        {
            s->sub_stage = 21;
        }
        break;

    case 19:
        if (d->originValue == d->new_alloc_region)
        {
            d->originValue = 0;

            if (!hwgc_access(s, d->free_list_ptr + 0x38, &d->originValue, 8, true))
                return;
        }

        s->sub_stage = 20;
        break;

    case 20:
        d->active_node_ids -= 1;

        if (!hwgc_access(s, d->free_list_ptr + 0x10, &d->active_node_ids, 4, true))
            return;

        s->sub_stage = 21;
        break;

    case 21:
        if (d->allocate_free_sel)
            hwgc_goto_stage(s, STAGE_NEW_GC_ALLOC, 4);
        else
            hwgc_goto_stage(s, STAGE_NEW_GC_ALLOC, 16);
        return;

    default:
        hwgc_goto_stage(s, STAGE_ALLOC_FREE_REGION, 0);
        break;
    }
}

static void stage_copy_function(HWGCDevState *s)
{
    HWGCStageData *d = &s->stageData;

    switch (s->sub_stage)
    {
    case 0:
        if (d->i >= d->size)
        {
            hwgc_goto_stage(s, STAGE_COPY2SURVIVOR, 20);
            return;
        }

        if (!hwgc_access(s, d->from_obj + d->i * 8, &d->data, 8, false))
            return;

        s->sub_stage = 1;
        break;

    case 1:
        if (!hwgc_access(s, d->to_obj + d->i * 8, &d->data, 8, true))
            return;

        d->i++;
        s->sub_stage = 0;
        break;

    default:
        hwgc_goto_stage(s, STAGE_COPY, 0);
        break;
    }
}

static void stage_trace_function(HWGCDevState *s)
{
    HWGCStageData *d = &s->stageData;

    switch (s->sub_stage)
    {
    case 0:
        d->scanning_in_young = (int8_t)(d->dest_attr >> 8) == 0;

        if (d->lh < 0)
        {
            uintptr_t len_addr;

            d->end = d->common_oop_array_length % d->pars.chunkSize;
            len_addr = d->to_obj + (d->pars.useCompressedKlassPointers ? 12 : 16);

            if (!hwgc_access(s, len_addr, &d->end, 4, true))
                return;

            s->sub_stage = 1;
        }
        else
        {
            if (!hwgc_access(s, d->klass_ptr + 160, &d->vtable_len, 4, false))
                return;

            s->sub_stage = 5;
        }
        break;

    case 1:
        if (d->common_oop_array_length > d->end)
        {
            if (!hwgc_access(s, d->pars.taskQueueBottomAddr, &d->array_localBot, 4, false))
                return;

            s->sub_stage = 2;
        }
        else
        {
            s->sub_stage = 4;
        }
        break;

    case 2:
    {
        uintptr_t pushData = d->from_obj + 0x2;

        if (!hwgc_access(s,
                         d->pars.taskQueueElemsBase + d->array_localBot * 8,
                         &pushData,
                         8,
                         true))
            return;

        s->sub_stage = 3;
        break;
    }

    case 3:
        d->array_localBot = (d->array_localBot + 1) & ((1 << 17) - 1);

        if (!hwgc_access(s, d->pars.taskQueueBottomAddr, &d->array_localBot, 4, true))
            return;

        s->sub_stage = 4;
        break;

    case 4:
    {
        uintptr_t base;
        uintptr_t low;
        uintptr_t high;
        size_t oop_size = d->pars.useCompressedOops ? 4 : 8;

        base = d->to_obj + (d->pars.useCompressedKlassPointers ? 16 : 24);
        low = base;
        high = base + d->end * oop_size;

        d->p = base;
        d->q = base + d->common_oop_array_length * oop_size;

        if (d->p < low)
            d->p = low;

        if (d->q > high)
            d->q = high;

        d->previous = STAGE_TRACE_PLUS;
        d->previous_sub_stage = 0;

        d->done_to = STAGE_COMMON_OOP;
        d->doneto_sub_stage = 4;

        hwgc_goto_stage(s, STAGE_TRACE_PLUS, 0);
        break;
    }

    case 5:
        if (!hwgc_access(s, d->klass_ptr + 296, &d->originValue, 8, false))
            return;

        s->sub_stage = 6;
        break;

    case 6:
    {
        int itable_len = d->originValue >> 32;
        int nonStaticOopMapSize = (int)d->originValue;

        d->start_map = (uintptr_t)((uintptr_t *)(d->klass_ptr + 464) +
                                   d->vtable_len + itable_len);
        d->end_map = d->start_map + nonStaticOopMapSize * 8;

        s->sub_stage = 7;
        break;
    }

    case 7:
        if (d->start_map < d->end_map)
        {
            d->end_map -= 8;

            if (!hwgc_access(s, d->end_map, &d->originValue, 8, false))
                return;

            s->sub_stage = 8;
        }
        else
        {
            s->sub_stage = 9;
        }
        break;

    case 8:
    {
        int offset = (int)d->originValue;
        int count = d->originValue >> 32;

        d->p = d->to_obj + offset;
        d->q = d->p + count * (d->pars.useCompressedOops ? 4 : 8);

        d->previous = STAGE_TRACE_DEC;
        d->previous_sub_stage = 0;

        d->done_to = STAGE_TRACE;
        d->doneto_sub_stage = 7;

        hwgc_goto_stage(s, STAGE_TRACE_DEC, 0);
        break;
    }

    case 9:
        if (d->kid == 2)
        {
            if (!hwgc_access(s, d->from_obj + 40, &d->staticCount, 4, false))
                return;

            s->sub_stage = 10;
        }
        else if (d->kid == 1)
        {
            d->i = 0;
            s->sub_stage = 11;
        }
        else
        {
            hwgc_goto_stage(s, STAGE_COMMON_OOP, 4);
        }
        break;

    case 10:
        d->p = d->to_obj + 184;
        d->q = d->p + d->staticCount * (d->pars.useCompressedOops ? 4 : 8);

        d->previous = STAGE_TRACE_PLUS;
        d->previous_sub_stage = 0;

        d->done_to = STAGE_COMMON_OOP;
        d->doneto_sub_stage = 4;

        hwgc_goto_stage(s, STAGE_TRACE_PLUS, 0);
        break;

    case 11:
        if (d->i != 3)
        {
            uint32_t discovered_offset;
            uint32_t referent_offset;

            if (d->pars.useCompressedKlassPointers & d->pars.useCompressedOops)
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

            d->src = d->i == 1
                       ? d->from_obj + referent_offset
                       : d->from_obj + discovered_offset;
            d->dest = d->i == 1
                        ? d->to_obj + referent_offset
                        : d->to_obj + discovered_offset;

            d->i++;

            d->previous = STAGE_TRACE;
            d->previous_sub_stage = 11;

            hwgc_goto_stage(s, STAGE_DO_OOP_WORK, 0);
        }
        else
        {
            d->i = 0;
            hwgc_goto_stage(s, STAGE_COMMON_OOP, 4);
        }
        break;

    default:
        hwgc_goto_stage(s, STAGE_TRACE, 0);
        break;
    }
}

static void stage_trace_plus_function(HWGCDevState *s)
{
    HWGCStageData *d = &s->stageData;
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

static void stage_trace_dec_function(HWGCDevState *s)
{
    HWGCStageData *d = &s->stageData;
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

static void stage_do_oop_work_function(HWGCDevState *s)
{
    HWGCStageData *d = &s->stageData;

    switch (s->sub_stage)
    {
    case 0:
        if (!hwgc_access(s, d->src, &d->heap_oop, 8, false))
            return;

        s->sub_stage = 1;
        break;

    case 1:
    {
        uintptr_t region_attr_ptr;

        if (d->pars.useCompressedOops)
            d->heap_oop = (uint32_t)d->heap_oop;

        if (d->heap_oop == 0)
        {
            s->sub_stage = 9;
            break;
        }

        if (d->pars.useCompressedOops)
            d->heap_oop = d->pars.compressedOopBase +
                          (d->heap_oop << d->pars.compressedOopShift);

        region_attr_ptr = d->pars.regionAttrBiasedBase +
                          (d->heap_oop >> d->pars.regionAttrShiftBy) * 2;

        if (!hwgc_access(s, region_attr_ptr, &d->region_attr, 2, false))
            return;

        s->sub_stage = 2;
        break;
    }

    case 2:
    {
        int8_t region_attr_type = d->region_attr >> 8;

        if (region_attr_type >= 0)
        {
            if (!hwgc_access(s, d->pars.taskQueueBottomAddr, &d->array_localBot, 4, false))
                return;

            s->sub_stage = 7;
        }
        else if (((d->dest ^ d->heap_oop) >> d->pars.logOfHRGrainBytes) != 0)
        {
            if (region_attr_type == -2)
                s->sub_stage = 3;
            else
                s->sub_stage = 6;
        }
        else
        {
            s->sub_stage = 9;
        }
        break;
    }

    case 3:
        d->region =
            (d->heap_oop -
             ((uintptr_t)d->pars.heapRegionBias << d->pars.heapRegionShiftBy)) >>
            d->pars.logOfHRGrainBytes;

        if (!hwgc_access(s,
                         d->pars.humogousReclaimCandidateBoolBase + d->region,
                         &d->bool_base_value,
                         1,
                         false))
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

        if (!hwgc_access(s,
                         d->pars.humogousReclaimCandidateBoolBase + d->region,
                         &d->bool_base_value,
                         1,
                         true))
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
        {
            hwgc_return_previous(s);
        }
        else
        {
            d->aop_region_attr = d->region_attr;
            d->aop_dest = d->dest;

            hwgc_goto_stage(s, STAGE_AOP_WORK, 0);
        }
        break;

    case 7:
    {
        uintptr_t writeElems = d->dest + (d->pars.useCompressedOops ? 1 : 0);

        if (!hwgc_access(s,
                         d->pars.taskQueueElemsBase + d->array_localBot * 8,
                         &writeElems,
                         8,
                         true))
            return;

        s->sub_stage = 8;
        break;
    }

    case 8:
        d->array_localBot = (d->array_localBot + 1) & ((1 << 17) - 1);

        if (!hwgc_access(s, d->pars.taskQueueBottomAddr, &d->array_localBot, 4, true))
            return;

        s->sub_stage = 9;
        break;

    case 9:
        hwgc_return_previous(s);
        break;

    default:
        hwgc_goto_stage(s, STAGE_DO_OOP_WORK, 0);
        break;
    }
}

static void stage_aop_work_function(HWGCDevState *s)
{
    HWGCStageData *d = &s->stageData;

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

        if (!hwgc_access(s, d->pars.cardTablePtr + 0x40, &d->byte_map_base, 8, false))
            return;

        d->res = d->byte_map_base + (d->aop_dest >> 9);
        d->card_index = d->res - d->byte_map;

        s->sub_stage = 1;
        break;

    case 1:
        if (!hwgc_access(s,
                         d->pars.parScanThreadStatePtr + 0x1b0,
                         &d->last_index,
                         8,
                         false))
            return;

        if (d->card_index == d->last_index)
        {
            hwgc_return_previous(s);
            return;
        }

        if (!hwgc_access(s,
                         d->pars.parScanThreadStatePtr + 0x48,
                         &d->index,
                         8,
                         false))
            return;

        if (!hwgc_access(s,
                         d->pars.parScanThreadStatePtr + 0x58,
                         &d->buffer,
                         8,
                         false))
            return;

        d->index /= 8;

        s->sub_stage = 2;
        break;

    case 2:
        if (d->index == 0)
        {
            d->old_node = 0;

            if (d->buffer != 0)
            {
                uintptr_t zero = 0;

                d->old_node = d->buffer - 0x10;

                if (!hwgc_access(s, d->old_node, &zero, 8, true))
                    return;
            }

            s->sub_stage = 6;
        }
        else
        {
            s->sub_stage = 3;
        }
        break;

    case 3:
        d->index--;

        if (!hwgc_access(s, d->buffer + d->index * 8, &d->res, 8, true))
            return;

        s->sub_stage = 4;
        break;

    case 4:
        d->index *= 8;

        if (!hwgc_access(s,
                         d->pars.parScanThreadStatePtr + 0x48,
                         &d->index,
                         8,
                         true))
            return;

        s->sub_stage = 5;
        break;

    case 5:
        if (!hwgc_access(s,
                         d->pars.parScanThreadStatePtr + 0x1b0,
                         &d->card_index,
                         8,
                         true))
            return;

        hwgc_return_previous(s);
        break;

    case 6:
        if (!hwgc_access(s,
                         d->pars.parScanThreadStatePtr + 0x20,
                         &d->node_allocator_ptr,
                         8,
                         false))
            return;

        s->sub_stage = 7;
        break;

    case 7:
        d->new_top = 0;

        if (!hwgc_access(s, d->node_allocator_ptr + 0x80, &d->node, 8, false))
            return;

        if (d->node != 0)
        {
            if (!hwgc_access(s, d->node + 0x8, &d->new_top, 8, false))
                return;
        }

        s->sub_stage = 8;
        break;

    case 8:
        if (!hwgc_access(s, d->node_allocator_ptr + 0x80, &d->new_top, 8, true))
            return;

        s->sub_stage = 9;
        break;

    case 9:
        if (d->node != 0)
        {
            uintptr_t zero = 0;

            if (!hwgc_access(s, d->node + 0x8, &zero, 8, true))
                return;

            s->sub_stage = 10;
        }
        else
        {
            ++enqueued_irq;

            d->softPars.par0 = d->node_allocator_ptr;

            s->wake_stage = STAGE_AOP_WORK;
            s->wake_sub_stage = 10;

            hwgc_goto_stage(s, STAGE_DEBUG, 0);

            if (qatomic_read(&s->status) & HWGC_STATUS_IRQ)
            {
                bql_lock();
                hwgc_raise_irq(s, ENQUEUE_FAILED_IRQ);
                bql_unlock();
            }

            return;
        }
        break;

    case 10:
        if (s->wake_stage == STAGE_AOP_WORK &&
            s->wake_sub_stage == 10 &&
            d->softPars.par0 == d->node_allocator_ptr)
        {
            d->node = d->softPars.res;
        }

        d->buffer = d->node + 0x10;

        if (!hwgc_access(s,
                         d->pars.parScanThreadStatePtr + 0x58,
                         &d->buffer,
                         8,
                         true))
            return;

        s->sub_stage = 11;
        break;

    case 11:
        if (!hwgc_access(s, d->node_allocator_ptr, &d->index, 8, false))
            return;

        d->originValue = d->index * 8;

        if (!hwgc_access(s,
                         d->pars.parScanThreadStatePtr + 0x48,
                         &d->originValue,
                         8,
                         true))
            return;

        if (d->old_node == 0)
            s->sub_stage = 3;
        else
            s->sub_stage = 12;
        break;

    case 12:
        if (!hwgc_access(s,
                         d->pars.parScanThreadStatePtr + 0x40,
                         &d->originValue,
                         8,
                         false))
            return;

        d->originValue += d->index;

        if (!hwgc_access(s,
                         d->pars.parScanThreadStatePtr + 0x40,
                         &d->originValue,
                         8,
                         true))
            return;

        s->sub_stage = 13;
        break;

    case 13:
        if (!hwgc_access(s,
                         d->pars.parScanThreadStatePtr + 0x30,
                         &d->originValue,
                         8,
                         false))
            return;

        if (!hwgc_access(s, d->old_node + 0x8, &d->originValue, 8, true))
            return;

        if (!hwgc_access(s,
                         d->pars.parScanThreadStatePtr + 0x30,
                         &d->old_node,
                         8,
                         true))
            return;

        s->sub_stage = 14;
        break;

    case 14:
        if (!hwgc_access(s,
                         d->pars.parScanThreadStatePtr + 0x38,
                         &d->originValue,
                         8,
                         false))
            return;

        if (d->originValue == 0)
        {
            if (!hwgc_access(s,
                             d->pars.parScanThreadStatePtr + 0x38,
                             &d->old_node,
                             8,
                             true))
                return;
        }

        s->sub_stage = 3;
        break;

    default:
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
        stage_fetch_function(s);
        break;

    case STAGE_DISPATCH:
        stage_dispatch_function(s);
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

    case STAGE_ALLOC_FREE_REGION:
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
    default:
        break;
    }
}
