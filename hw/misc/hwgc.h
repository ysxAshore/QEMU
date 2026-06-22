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
    STAGE_ARRAY_PROCESS = 2,
    STAGE_OOP_PROCESS = 3,
    STAGE_COPY2SURVIVOR = 4,
    STAGE_TRACE = 5,
    STAGE_ALLOCATE = 6,
    STAGE_PAR_ALLOCATE_DURING_GC = 7,
    STAGE_ATTEMPT_ALLOC = 8,
    STAGE_NEW_GC_ALLOC = 9,
    STAGE_ALLOCATE_FREE = 10,
    STAGE_PAR_ALLOCATE = 11,
    STAGE_PAR_ALLOCATE_IML = 12,
    STAGE_DO_OOP_WORK = 13,
    STAGE_AOP_WORK = 14,
    STAGE_NODE_ALLOCATE = 15,
    STAGE_DONE = 16
}

struct HWGCParameters
{
    uint32_t chunkSize;
    uint32_t ageThreshold;
    uint32_t heapRegionBias;
    uint32_t regionAttrShiftBy;
    uint32_t heapRegionShiftBy;
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
    uint64_t miss_va;
    uint32_t miss_access;
    uint64_t fill_va;
    hwaddr fill_pa;

    struct HWGCParameters pars;

    uintptr_t task;
    uintptr_t from_obj;
    uintptr_t to_obj;

    uintptr_t partial_m_value;
    uint partial_from_length;
    uint partial_to_length;
    uintptr_t heap_region;
    uint heap_region_type;

    bool isArray;
    uint stepIndex;
    uint stepNcreate;
    uint arrayLength;
    uint partial_start;
    bool scanning_in_young;

    uintptr_t offset;
    uintptr_t common_m_value;
    uintptr_t src_region_attr_ptr;
    uint16_t src_region_attr;

    uintptr_t klass_ptr;
    uint lh, kid, size, age;
    bool dest_attr_valid;
    uint dest_attr_cache;
    int8_t src_region_attr_type;
    uint16_t dest_attr;
    uintptr_t dest_attr_ptr;
    uintptr_t monitor_markWord;
    int plab_idx;
    uintptr_t buffer_ptr[2];
    uintptr_t buffer[2];
    uintptr_t plab_top[2];
    uintptr_t plab_end[2];
    bool plab_buffer_valid[2];
    bool plab_top_end_valid[2];
    uintptr_t obj_ptr, forward_ptr;
    uintptr_t writeSrcMW;
    uintptr_t region_bottom, region_hard_end;
    uint idx;

    uintptr_t aop_region_attr_ptr;
    uintptr_t aop_p;
}

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

    struct HWGCStageData stageData;

    HWGCTLBEntry tlb[HWGC_TLB_SIZE];
};