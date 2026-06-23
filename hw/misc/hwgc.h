#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qemu/timer.h"
#include "qemu/main-loop.h"
#include "qemu/module.h"
#include "qom/object.h"
#include "hw/pci/pci.h"
#include "hw/pci/msi.h"
#include "system/address-spaces.h"

#define TYPE_HWGC_DEV "hwgc"
typedef struct HWGCDevState HWGCDevState;
DECLARE_INSTANCE_CHECKER(HWGCDevState, HWGC_DEV, TYPE_HWGC_DEV)

#define HWGC_VENDOR_ID PCI_VENDOR_ID_QEMU
#define HWGC_DEVICE_ID 0x0308

#define REG_STATUS 0x0
#define REG_IRQ_STATUS 0x4
#define REG_IRQ_CLEAR 0x8
#define REG_CMD 0x0c

#define REG_PAR0 0x10  // AgeThreshold ++ ChunkSize
#define REG_PAR1 0x18  // HeapRegionShiftBy ++ Bias
#define REG_PAR2 0x20  // LogOfHRGrainBytes ++ RegionAttrShiftBy
#define REG_PAR3 0x28  // CompressedFlag ++ TaskQueueBottom
#define REG_PAR4 0x30  // StepperOffset
#define REG_PAR5 0x38  // YoungWordsBase
#define REG_PAR6 0x40  // RegionAttrBase
#define REG_PAR7 0x48  // PlabAllocatorPTr
#define REG_PAR8 0x50  // RegionAttrBiasedBase
#define REG_PAR9 0x58  // HeapRegionBiasedBase
#define REG_PAR10 0x60 // ParScanThreadStatePtr
#define REG_PAR11 0x68 // TaskQueueElemsBase
#define REG_PAR12 0x70 // HumoungousReclaimBoolBase
#define REG_PAR13 0x78 // CardTablePtr
#define REG_PAR14 0x80 // G1h
#define REG_PAR15 0x88 // IntArrayKlassObj
#define REG_PAR16 0x90 // ObjectKlass
#define REG_PAR17 0x98 // LockPtr
#define REG_PAR18 0xa0 // Thread
#define REG_PAR19 0xa8 // DummyRegion
#define REG_PAR20 0xb0 // CompressedOopBase
#define REG_PAR21 0xb8 // CompressedKlassPointerBase

/*
 * IRQ define
 * BufferNode::allocate                    -> param: allocator_ptr(uintptr_t); return_value: node(uintptr_t)
 * expand_single_region                    -> param: node_index(uint)        ; return_value: success(bool)
 * ((GrowableArray<HeapRegion *> *))->grow -> param: grow_array_ptr(uintptr_t), len(int); return_value: no
 * ((Mutex *)) ->unlock                    -> param: lock_ptr(uintptr_t); return_value: no
 * TLB_MISS                                -> param: MISS_VA(uintptr_t), MISS_ACCESS(uint); return_value: TLB_FILL_VA(uintptr_t), TLB_FILL_PA(uintptr_t)
 * DONE                                    -> param: no; return_value: no
 * ERROR                                   -> param: no; return_value: no
 */
#define REG_IRQ_PAR0 0xc0
#define REG_IRQ_PAR1 0xc8
#define REG_IRQ_RES0 0xd0
#define REG_IRQ_RES1 0xd8

/* Command bits */
#define CMD_START 0x1
#define CMD_CONTINUE 0x2
#define CMD_RESET 0x4

/* Status bits */
#define ST_BUSY 0x1
#define ST_DONE 0x2
#define ST_WAIT_TLB 0x4
#define ST_WAIT_GROW 0x8
#define ST_WAIT_EXPAND 0x10
#define ST_WAIT_ALLOCATE 0x20
#define ST_WAIT_WAKE 0x40
#define ST_ERROR 0x80
#define ST_IRQ_EN 0x100

/* IRQ bits*/
#define IRQ_TLB_MISS 0x1
#define IRQ_DONE 0x2
#define IRQ_GORW 0x4
#define IRQ_EXPAND 0x8
#define IRQ_ALLOCATE 0x10
#define IRQ_WAKE 0x20
#define IRQ_ERROR 0x40

/* Device access type reported to driver */
#define ACCESS_READ 1
#define ACCESS_WRITE 2

/* Local page constants (independent of target arch macros) */
#define HWGC_PAGE_SHIFT 14 // linux里这里是14
#define HWGC_PAGE_SIZE (1ULL << HWGC_PAGE_SHIFT)
#define HWGC_PAGE_MASK (HWGC_PAGE_SIZE - 1)

#define HWGC_TLB_SIZE 512 * 1024

typedef struct
{
    bool valid;
    uint64_t va_page;
    hwaddr pa_page;
} HWGCTLBEntry;

enum HWGCStage
{
    STAGE_IDLE = 0,
    STAGE_FETCH = 1,
    STAGE_PARTIAL_ARRAY = 2,
    STAGE_COMMON_OOP = 3,
    STAGE_COPY2SURVIVOR = 4,
    STAGE_ALLOC = 5,
    STAGE_ALLOCATE_DIRECT = 6,
    STAGE_ALLOCATE_DURING_GC = 7,
    STAGE_PAR_ALLOCATE_IML = 8,
    STAGE_PAR_ALLOCATE = 9,
    STAGE_COPY = 10,
    STAGE_TRACE = 11,
    STAGE_TRACE_PLUS = 12,
    STAGE_TRACE_DEC = 13,
    STAGE_DO_OOP_WORK = 14,
    STAGE_AOP_WORK = 15,
    STAGE_DONE = 16,

    STAGE_ATTEMPT_ALLOC = 17,
    STAGE_NEW_GC_ALLOC = 18,
    STAGE_ALLOCATE_FREE = 19
};

struct HWGCParameters
{
    uint32_t chunkSize;
    uint32_t ageThreshold;
    uint32_t heapRegionBias;
    uint32_t heapRegionShiftBy;
    uint32_t regionAttrShiftBy;
    uint32_t logOfHRGrainBytes;
    uint64_t stepperOffset;
    uint64_t youngWordsBase;
    uint64_t regionAttrBase;
    uint64_t plabAllocatorPtr;
    uint64_t regionAttrBiasedBase;
    uint64_t heapRegionBiasedBase;
    uint64_t pss;
    uint32_t localBot;
    uint64_t taskQueueElemsBase;
    uint64_t humogousReclaimCandidateBoolBase;
    uint64_t cardTablePtr;
    uint64_t g1h;
    uint64_t intArrayKlassObj;
    uint64_t objectKlass;
    uint64_t lockPtr;
    uint64_t thread;
    uint64_t dummyRegion;
    uint64_t compressedOopBase;
    uint64_t compressedKlassPointerBase;
    uint8_t compressedOopShift;
    uint8_t compressedKlassPointerShift;
    uint8_t useCompressedOops;
    uint8_t useCompressedKlassPointers;
};

struct HWGCStageData
{
    struct HWGCParameters pars;

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
    uint32_t dest_attr_cache;
    uint16_t dest_attr;

    uintptr_t dest_attr_ptr;
    uintptr_t monitor_markWord;
    uintptr_t from_region;

    uintptr_t buffer_temp;
    uintptr_t buffer;
    uintptr_t region_top;
    uintptr_t region_end;
    uintptr_t region_bottom;
    uintptr_t region_hard_end;

    uintptr_t new_mark;
    uintptr_t writeSrcMW;
    uintptr_t forward_ptr;

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
    bool plab_refill_failed;

    uintptr_t region_ptr;
    uintptr_t alloc_region;

    uintptr_t card_table_ptr;
    uintptr_t byte_map_base;
    uintptr_t first;
    uintptr_t last;

    uintptr_t remaining;
    uint buf[256];
    uintptr_t offset30;
    uintptr_t offset38;

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
};

struct HWGCDevState
{
    PCIDevice pdev;
    MemoryRegion mmio;

    QemuThread worker;
    QemuMutex lock;
    QemuCond cond;
    QEMUTimer *tick_timer;

    bool thread_stop;
    bool timer_running;
    bool tick_pending;

    uint64_t period_ns;

    uint32_t status;
    uint32_t irq_status;
    enum HWGCStage stage;
    int sub_stage;
    uint32_t irq_source;

    uint64_t irq_par0;
    uint64_t irq_par1;
    uint64_t irq_res0;
    uint64_t irq_res1;

    struct HWGCStageData stageData;

    HWGCTLBEntry tlb[HWGC_TLB_SIZE];
};