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
    idx = xor_tlb_hash(va_page);

    s->tlb[idx].valid = true;
    s->tlb[idx].va_page = va_page;
    s->tlb[idx].pa_page = pa_page;

    printf("[xor] TLB insert: idx=%u va_page=0x%016" PRIx64
           " pa_page=0x%016" HWADDR_PRIx "\n",
           idx, va_page, pa_page);
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

static void hwgc_raise_irq_from_worker(XorTLBDevState *s, uint32_t bits)
{
    qatomic_or(&s->irq_status, bits);

    if (!(qatomic_read(&s->status) & ST_IRQ_EN))
        return;

    bql_lock();
    if (xor_msi_enabled(s))
        msi_notify(&s->pdev, 0);
    else
        pci_set_irq(&s->pdev, 1);
    bql_unlock();
}

static void hwgc_lower_irq_from_mmio(XorTLBDevState *s, uint32_t bits)
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
    qemu_mutex_lock(&s->lock);

    s->miss_va = va;
    s->miss_access = access;
    s->timer_running = false;
    qatomic_or(&s->status, ST_WAIT_TLB);

    qemu_mutex_unlock(&s->lock);

    xor_raise_irq_from_worker(s, IRQ_TLB_MISS);
}

/*
 * Device-side translation: VA -> TLB lookup -> PA.
 * If miss, raise IRQ and pause. The driver will fill REG_TLB_FILL_* and
 * write CMD_CONTINUE, after which the same stage is retried.
 */
static bool hwgc_translate(HWGCDevState *s, uint64_t va, uint32_t access, hwaddr *pa)
{
    bool hit;

    qemu_mutex_lock(&s->lock);
    hit = hwgc_tlb_lookup_locked(s, va, pa);
    qemu_mutex_unlock(&s->lock);

    if (!hit)
    {
        printf("translate not hit\n");
        hwgc_pause_for_tlb_miss(s, va, access);
        return false;
    }

    return true;
}

static bool hwgc_access(XorTLBDevState *s, uint64_t va, void *val, uint size, bool write)
{
    hwaddr pa;
    MemTxResult tx;

    int align = size - 1;

    // demo check va必须8B对齐 且访问必须在一页内
    if ((va & align) != 0 || ((va & (HWGC_PAGE_SIZE - 1)) > HWGC_PAGE_SIZE - size))
    {
        printf("va address error\n");
        qemu_mutex_lock(&s->lock);
        qatomic_or(&s->status, ST_ERROR);
        s->timer_running = false;
        qemu_mutex_unlock(&s->lock);
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
        qemu_mutex_lock(&s->lock);
        qatomic_or(&s->status, ST_ERROR);
        s->timer_running = false;
        qemu_mutex_unlock(&s->lock);
        hwgc_raise_irq_from_worker(s, IRQ_ERROR);
        return false;
    }

    return true;
}

static bool hwgc_cmpxchg(HWGCDevState *hwgc, uintptr_t vaddr, uint64_t old_val, uint64_t new_val, int size, void *return_value)
{
    // demo check va必须8B对齐 且访问必须在一页内
    if ((vaddr & 7) != 0 || ((vaddr & (HWGC_PAGE_SIZE - 1)) > HWGC_PAGE_SIZE - sizeof(uint64_t)))
    {
        printf("va address error\n");
        qemu_mutex_lock(&s->lock);
        qatomic_or(&s->status, ST_ERROR);
        s->timer_running = false;
        qemu_mutex_unlock(&s->lock);
        hwgc_raise_irq_from_worker(s, IRQ_ERROR);
        return false;
    }

    hwaddr paddr;
    if (!hwgc_translate(hwgc, vaddr, true, &paddr))
        return false;

    RCU_READ_LOCK_GUARD();

    hwaddr xlat = paddr;
    hwaddr len = size;
    MemoryRegion *mr = address_space_translate(&address_space_memory, paddr, &xlat, &len, true, MEMTXATTRS_UNSPECIFIED);
    if (!memory_access_is_direct(mr, true, MEMTXATTRS_UNSPECIFIED) || len < size)
    {
        printf("memory region not is ram\n");

        qemu_mutex_lock(&s->lock);
        qatomic_or(&s->status, ST_ERROR);
        s->timer_running = false;
        qemu_mutex_unlock(&s->lock);

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

        qemu_mutex_lock(&s->lock);
        qatomic_or(&s->status, ST_ERROR);
        s->timer_running = false;
        qemu_mutex_unlock(&s->lock);

        hwgc_raise_irq_from_worker(s, IRQ_ERROR);
        return false;
    }
}

static void stage_fetch_function(HWGCDevState *s)
{
    assert(s->sub_stage == 0);
    if (s->stageData.pars.localBot == 0)
    {
        qemu_mutex_lock(&s->lock);
        s->stage = STAGE_DONE;
        s->timer_running = false;
        qatomic_and(&s->status, ~ST_BUSY);
        qatomic_or(&s->status, ST_DONE);
        qemu_mutex_unlock(&s->lock);

        hwgc_raise_irq_from_worker(s, IRQ_DONE);
    }
    else
    {
        uint fetch_idx = s->stageData.pars.localBot - 1;
        uintptr_t vaddr = s->stageData.pars.taskQueueElemsBase + fetch_idx * 8;
        if (!hwgc_access(s, vaddr, &s->stageData.task, 8, false))
            return;
        s->stageData.pars.localBot = fetch_idx;
        if ((s->stageData.task & 0x3) == 0x2)
        {
            s->stageData.task -= 0x2;
            s->stage = STAGE_ARRAY_PROCESS;
        }
        else
        {
            s->stageData.task -= (s->stageData.task & 0x3);
            s->stage = STAGE_OOP_PROCESS;
        }
    }
}

static void stage_array_function(HWGCDevState *s)
{
    switch (s->sub_stage)
    {
    case 0:
        s->stageData.from_obj = s->stageData.task;

        if (!hwgc_access(s, s->stageData.from_obj, &s->stageData.partial_m_value, 8, false))
            return;

        s->sub_stage = 1;
        break;

    case 1:
        s->stageData.to_obj = s->stageData.partial_m_value & ~0x3;
        uintptr_t length_addr = s->stageData.from_obj + (s->stageData.pars.useCompressedKlassPointers ? 12 : 16);

        if (!hwgc_access(s, length_addr, &s->stageData.partial_from_length, 4, false))
            return;

        s->sub_stage = 2;
        break;

    case 2:
        uintptr_t length_addr = s->stageData.to_obj + (s->stageData.pars.useCompressedKlassPointers ? 12 : 16);

        if (!hwgc_access(s, length_addr, &s->stageData.partial_to_length, 4, false))
            return;

        s->sub_stage = 3;
        break;

    case 3:
        s->stageData.stepIndex = s->stageData.partial_to_length + s->stageData.pars.chunkSize;
        uintptr_t length_addr = s->stageData.to_obj + (s->stageData.pars.useCompressedKlassPointers ? 12 : 16);

        if (!hwgc_access(s, length_addr, &s->stageData.stepIndex, 4, true))
            return;

        s->sub_stage = 4;
        break;

    case 4:
        uint32_t task_num = s->stageData.partial_to_length / s->stageData.pars.chunkSize;
        uint32_t remaining_tasks = (s->stageData.partial_from_length - s->stageData.partial_to_length) / s->stageData.pars.chunkSize;
        uint32_t task_limit = (uint32_t)s->stageData.pars.stepperOffset;
        uint32_t task_fanout = (uint32_t)(s->stageData.pars.stepperOffset >> 32);
        uint32_t max_pending = (task_fanout - 1) * task_num + 1;
        uint32_t pending = MIN(max_pending, MIN(remaining_tasks, task_limit));
        s->stageData.stepNcreate = MIN(task_fanout, MIN(remaining_tasks, task_limit + 1) - pending);
        s->sub_stage = 5;
        break;

    case 5:
        uintptr_t heap_region_ptr = s->stageData.pars.heapRegionBiasedBase + (s->stageData.to_obj >> s->stageData.pars.heapRegionShiftBy) * 8;

        if (!hwgc_access(s, heap_region_ptr, &s->stageData.heap_region, 8, false))
            return;

        s->sub_stage = 6;
        break;

    case 6:
        if (!hwgc_access(s, s->stageData.heap_region + 0xbc, &s->stageData.heap_region_type, 4, false))
            return;

        s->stageData.isArray = true;
        s->stageData.arrayLength = s->stageData.stepIndex;
        s->stageData.partial_start = s->stageData.partial_to_length;
        s->stageData.scanning_in_young = (s->stageData.heap_region_type & 0x2) != 0;

        s->stage = STAGE_TRACE;
        s->sub_stage = 0;
        break;

    default:
        break;
    }
}

static void stage_oop_function(HWGCDevState *s)
{

    switch (s->sub_stage)
    {
    case 0:
        if (!hwgc_access(s, s->stageData.task, &s->stageData.offset, 8, false))
            return;

        s->sub_stage = 1;
        break;

    case 1:
        if (s->stageData.pars.useCompressedOops)
        {
            uint32_t narrow_oop = (uint32_t)s->stageData.offset;
            if (narrow_oop == 0)
                s->stageData.from_obj = 0;
            else
                s->stageData.from_obj = (uintptr_t)s->stageData.pars.compressedOopBase + ((uintptr_t)narrow_oop << s->stageData.pars.compressedOopShift);
        }
        else
            s->stageData.from_obj = s->stageData.offset;

        if (!hwgc_access(s, s->stageData.from_obj, &s->stageData.common_m_value, 8, false))
            return;

        s->sub_stage = 2;
        break;

    case 2:
        s->stageData.region_attr_ptr = hwgc->stageData.pars.regionAttrBiasedBase + (s->stageData.from_obj >> s->stageData.pars.regionAttrShiftBy) * 2;
        if (!hwgc_access(s, s->stageData.region_attr_ptr, &s->stageData.src_region_attr, 2, false))
            return;

        if ((int8_t)(s->stageData.src_region_attr >> 8) < 0)
        {
            s->stage = STAGE_FETCH;
            s->sub_stage = 0;
        }
        else
            s->sub_stage = 3;

        break;

    case 3:
        if ((s->stageData.common_m_value & 0x3) == 0x3)
        {
            s->stageData.to_obj = s->stageData.common_m_value & ~0x3;
            s->sub_stage = 4;
        }
        else
        {
            s->stage = STAGE_COPY2SURVIVOR;
            s->sub_stage = 0;
        }
        break;

    case 4:
        uintptr_t writeObj = s->stageData.to_obj;

        if (s->stageData.pars.useCompressedOops)
            writeObj = (s->stageData.to_obj - s->stageData.pars.compressedOopBase) >> s->stageData.pars.compressedOopShift;

        if (s->stageData.pars.useCompressedOops)
        {
            uint32_t narrow_writeObj = (uint32_t)writeObj;
            if (!hwgc_access(s, s->stageData.task, &narrow_writeObj, 4, true))
                return;
        }
        else
        {
            if (!hwgc_access(s, s->stageData.task, &writeObj, 8, true))
                return;
        }

        s->sub_stage = 5;
        break;

    case 5:
        if (((s->stageData.task ^ s->stageData.to_obj) >> s->stageData.pars.logOfHRGrainBytes) == 0)
        {
            s->stage = STAGE_FETCH;
            s->sub_stage = 0;
            return;
        }

        uintptr_t heap_region_ptr = s->stageData.pars.heapRegionBiasedBase + (s->stageData.task >> s->stageData.pars.heapRegionShiftBy) * 8;

        if (!hwgc_access(s, heap_region_ptr, &s->stageData.heap_region, 8, false))
            return;

        s->sub_stage = 6;
        break;

    case 6:
        if (!hwgc_access(s, s->stageData.heap_region + 0xbc, &s->stageData.heap_region_type, 4, false))
            return;

        bool type_is_young = (s->stageData.heap_region_type & 0x2) != 0;

        s->sub_stage = 0;
        if (!type_is_young)
        {
            s->stageData.aop_region_attr_ptr = s->stageData.pars.regionAttrBiasedBase + (s->stageData.to_obj >> s->stageData.pars.regionAttrShiftBy) * 2;
            s->stageData.aop_p = s->stageData.task;
            s->stage = STAGE_AOP_WORK;
        }
        else
            s->stage = STAGE_FETCH;
        break;
    }
}

static void stage_copy2survivor_function(HWGCDevState *s)
{
    static uintptr_t temp;
    switch (s->sub_stage)
    {
    case 0:
        if (!hwgc_access(s, s->stageData.from_obj + 8, &s->stageData.klass_ptr, 8, false))
            return;

        s->sub_stage = 1;
        break;

    case 1:
        if (s->stageData.pars.useCompressedKlassPointers)
        {
            uint32_t narrow_klass = (uint32_t)s->stageData.klass_ptr;
            s->stageData.klass_ptr = s->stageData.pars.compressedKlassPointerBase + ((uintptr_t)narrow_klass << s->stageData.pars.compressedKlassPointerShift);
        }

        if (!hwgc_access(s, s->stageData.klass_ptr + 8, &temp, 8, false))
            return;

        s->sub_stage = 2;
        break;

    case 2:
        s->stageData.lh = (int32_t)temp;
        s->stageData.kid = (int32_t)(temp >> 32);

        if (s->stageData.lh > 0 && s->stageData.lh & 0x1 == 0)
            s->stageData.size = s->stageData.lh >> 3;
        else if (s->stageData.lh < 0)
        {
            uintptr_t array_length_addr = s->stageData.from_obj + (s->stageData.pars.useCompressedKlassPointers ? 12 : 16);
            if (!hwgc_access(s, array_length_addr, &s->stageData.partial_from_length, 4, false))
                return;

            size_t temp = ((size_t)s->partial_from_length << (uint8_t)s->stageData.lh) + (uint8_t)(s->stageData.lh >> 16);
            s->stageData.size = (temp & 0x7) ? (temp >> 3) + 1 : (temp >> 3);
        }
        else
        {
            if (s->stageData.kid == 2)
            {
                if (!hwgc_access(s, s->stageData.from_obj + 0x20, &s->stageData.size, 4, false))
                    return;
            }
            else
                s->stageData.size = s->stageData.lh >> 3;
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
        int idx = s->stageData.plab_idx;
        if (!s->stageData.plab_buffer_valid[idx])
        {
            uintptr_t buffer_slot_addr = s->stageData.pars.plabAllocatorPtr + idx * 8;

            if (!hwgc_access(s, buffer_slot_addr, &s->stageData.buffer_ptr[idx], 8, false))
                return;

            s->sub_stage = 6;
        }
        else
            s->sub_stage = 7;

    case 6:
        int idx = s->stageData.plab_idx;

        if (!hwgc_access(s, s->stageData.buffer_ptr[idx], &s->stageData.buffer[idx], 8, false))
            return;

        s->stageData.plab_buffer_valid[idx] = true;

        s->sub_stage = 7;
        break;

    case 7:
    {
        uint32_t idx = s->stageData.plab_idx;

        if (!s->stageData.plab_top_end_valid[idx])
        {
            uintptr_t buffer = s->stageData.buffer[idx];

            if (!hwgc_access(s, buffer + 0x30, &s->stageData.plab_top[idx], 8, false))
                return;

            s->sub_stage = 8;
        }
        else
            s->sub_stage = 9;

        break;
    }

    case 8:
    {
        uint32_t idx = s->stageData.plab_idx;

        uintptr_t buffer = s->stageData.buffer[idx];

        if (!hwgc_access(s, buffer + 0x38, &s->stageData.plab_end[idx], 8, false))
            return;

        s->sub_stage = 9;
        s->stageData.plab_top_end_valid[idx] = true;

        break;
    }

    case 9:
    {
        uint32_t idx = s->stageData.plab_idx;

        uintptr_t top = s->stageData.plab_top[idx];
        uintptr_t end = s->stageData.plab_end[idx];

        if ((end - top) / 8 >= s->stageData.size)
        {
            uintptr_t buffer = s->stageData.buffer[idx];
            uintptr_t writeData = top + s->stageData.size * 8;

            if (!hwgc_access(s, buffer + 0x30, &writeData, 8, true))
                return;

            s->stageData.obj_ptr = top;
            s->stageData.plab_top[idx] = writeData;

            int other = idx == 0 ? 1 : 0;

            if (s->stageData.plab_top_end_valid[other] && s->stageData.buffer[0] == s->stageData.buffer[1])
                s->stageData.plab_top[other] = s->stageData.plab_top[idx];
        }
        else
        {
            s->stageData.obj_ptr = 0;
            s->stageData.plab_top_end_valid[idx] = false;

            int other = idx == 0 ? 1 : 0;

            if (s->stageData.plab_top_end_valid[other] && s->stageData.buffer[0] == s->stageData.buffer[1])
                s->stageData.plab_top_end_valid[other] = false;
        }

        s->sub_stage = 10;

        break;
    }

    case 10:
        if (s->stageData.obj_ptr == 0)
        {
            // send to allocate
        }
        else
            s->sub_stage = s->stageData.select_old ? 12 : 14;
        break;

    case 11:
        if (s->stageData.obj_ptr == 0)
        {
            s->stageData.plab_idx = 1;
            s->stageData.select_old = true;
            s->sub_stage = 5;
        }
        else
            s->sub_stage = 14;
        break;

    case 12:
        if (s->stageData.plab_refill_failed)
        {
            uint32_t zero = 0;
            uintptr_t threshold_addr = s->stageData.pars.parScanThreadStatePtr + 0x17c;
            if (!hwgc_access(s, threshold_addr, &zero, 4, true))
                return;
        }
        s->sub_stage = 13;

    case 13:
        uint8_t one = 1;

        if (!hwgc_access(s, s->stageData.dest_attr_ptr + 1, &one, 1, true))
            return;

        if (s->stageData.dest_attr_ptr == s->stageData.src_region_attr_ptr)
            s->stageData.src_region_attr = (s->stageData.src_region_attr & 0x00ff) | (1 << 8);
        if (s->stageData.dest_attr_ptr == s->stageData.pars.pss + 0x178)
            s->stageData.dest_attr_cache = (s->stageData.dest_attr_cache & 0xffff00ff) | (1 << 8);
        if (s->stageData.dest_attr_ptr == s->stageData.pars.pss + 0x17a)
            s->stageData.dest_attr_cache = (s->stageData.dest_attr_cache & 0xffffff) | (1 << 24);

        s->stageData.dest_attr = (s->stageData.dest_attr & 0x00ff) | (1 << 8);
        s->sub_stage = 14;

    case 14:
        s->stageData.writeSrcMW = (s->stageData.obj_ptr & ~0x3) | 0x3;

        uintptr_t return_value;
        if (!hwgc_cmpxchg(s, s->stageData.from_obj, s->stageData.common_m_value, s->stageData.writeSrcMW, 8, &return_value))
            return;

        if (return_value == s->stageData.common_m_value)
        {
            s->stageData.to_obj = s->stageData.obj_ptr;
            s->stageData.forward_ptr = 0;
            s->sub_stage = 17;
        }
        else
        {
            s->stageData.forward_ptr = return_value & ~0x3;
            s->sub_stage = 15;
        }
        break;

    case 15:
        uint32_t idx = s->stageData.plab_idx;
        uintptr_t buffer = s->stageData.buffer[idx];

        if (!hwgc_access_u64(s, buffer + 0x28, &s->stageData.region_bottom, 8, false))
            return;
        if (!hwgc_access_u64(s, buffer + 0x40, &s->stageData.region_hard_end, 8, false))
            return;

        if (s->stageData.obj_ptr >= s->stageData.region_bottom && s->stageData.obj_ptr < s->stageData.region_hard_end)
        {
            uint32_t other = idx == 1 ? 0 : 1;

            if (s->stageData.plab_top_end_valid[idx])
                s->stageData.plab_top[idx] = s->stageData.obj_ptr;

            if (s->stageData.plab_top_end_valid[other] && s->stageData.buffer[0] == s->stageData.buffer[1])
                s->stageData.plab_top_cache[other] = s->stageData.obj_ptr;

            uintptr_t buffer = s->stageData.buffer[idx];

            if (!hwgc_access_u64(s, buffer + 0x30, &s->stageData.obj_ptr, 8, true))
                return;

            s->stageData.to_obj = s->stageData.forward_ptr;
            s->stage = STAGE_OOP_PROCESS;
            s->sub_stage = 4;
        }
        else
            s->sub_stage = 16;
        break;

    case 16:
        uint words = s->stageData.size / 8;
        uintptr_t cur_klass = 0;
        uint header_words = s->stageData.pars.useCompressedKlassPointers ? 2 : 3;
        if (words >= header_words)
        {
            uint payload_size = words - header_words;
            uint32_t len = payload_size * 2;

            uintptr_t array_len_addr = s->stageData.obj_ptr + s->stageData.pars.useCompressedKlassPointers ? 12 : 16;
            if (!hwgc_access(s, array_len_addr, &s->stageData.dummy_array_len, 4, true))
                return;

            cur_klass = s->stageData.pars.intArrayKlassObj;
        }
        else if (words > 0)
            cur_klass = s->stageData.pars.objectKlass;

        uintptr_t mark = 0x1;
        if (!hwgc_access(s, s->stageData.obj_ptr, &mark, 8, true))
            return;

        if (s->stageData.pars.useCompressedKlassPointers)
        {
            uint32_t narrow_klass = (uint32_t)((cur_klass - s->stageData.pars.compressedKlassPointerBase) >> s->stageData.pars.compressedKlassPointerShift);
            if (!hwgc_access(s, s->stageData.obj_ptr + 0x8, &narrow_klass, 4, true))
                return;
        }
        else
        {
            if (!hwgc_access(s, s->stageData.obj_ptr + 0x8, &cur_klass, 8, true))
                return;
        }

        s->stageData.to_obj = s->stageData.forward_ptr;
        s->stage = STAGE_OOP_PROCESS;
        s->sub_stage = 4;
        break;

    case 17:
        int8_t dest_attr_type = (int8_t)(s->stageData.dest_attr >> 8);
        uintptr_t new_mark = s->stageData.common_m_value;
        uint16_t new_age = s->stageData.age + 1 < 15 ? s->stageData.age + 1 : s->stageData.age;

        if (dest_attr_type == 0)
        {
            if ((s->stageData.common_m_value & 0x1) == 0x0)
            {
                bool has_monitor = (s->stageData.common_m_value & 0x2) != 0;
                uintptr_t addr = has_monitor ? (s->stageData.common_m_value ^ 0x2) : s->stageData.common_m_value;
                if (!hwgc_access(s, s->stageData.monitor_ptr, &s->stageData.monitor_markWord, 8, false))
                    return;

                s->stageData.monitor_markWord = (s->stageData.monitor_markWord & ~((uintptr_t)0x1111 << 3)) | (((uintptr_t)new_age & 0xF) << 3);
                if (!hwgc_access(s, s->stageData.monitor_ptr, &s->stageData.monitor_markWord, 8, true))
                    return;
            }
            else
                new_mark = (s->stageData.common_m_value & ~((uintptr_t)0x1111 << 3)) | (((uintptr_t)new_age & 0x1111) << 3);
        }
        if (!hwgc_access_u64(s, s->stageData.to_obj, new_mark, 8, true))
            return;

        s->sub_stage = 18;
        s->stageData.idx = 1;

        break;

    case 18:
    {
        if (s->stageData.i < s->stageData.size)
        {
            uintptr_t src_addr = s->stageData.from_obj + s->stageData.idx * 8;
            uintptr_t dst_addr = s->stageData.to_obj + s->stageData.idx * 8;
            uintptr_t data;
            if (!hwgc_access_u64(s, src_addr, &data, 8, false))
                return;

            if (!hwgc_access_u64(s, dst_addr, &data, 8, true))
                return;

            s->stageData.idx++;
            s->sub_stage = 18;
        }
        else
            s->sub_stage = 19;

        break;
    }

    case 19:
    {
        if (s->stageData.kid == 4)
        {
            s->stage = STAGE_OOP_PROCESS;
            s->sub_stage = 4;
            return;
        }
        else
        {
            s->stageData.isArray = false;
            s->stageData.scanning_in_young = (int8_t)(s->stageData.dest_attr >> 8) == 0;
            s->stage = STAGE_TRACE;
            s->sub_stage = 0;
            return;
        }
    }

    default:
        break;
    }
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

        flush_hwgc_tlb(hwgc);

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