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

#define HWGC_TLB_SIZE 1024

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
    STAGE_ALLOCATE = 5,
    STAGE_PAR_ALLOCATE_DURING_GC = 6,
    STAGE_ATTEMPT_ALLOC = 7,
    STAGE_NEW_GC_ALLOC = 8,
    STAGE_ALLOCATE_FREE = 9,
    STAGE_PAR_ALLOCATE = 10,
    STAGE_PAR_ALLOCATE_IML = 11,
    STAGE_DO_OOP_WORK = 12,
    STAGE_AOP_WORK = 13,
    STAGE_NODE_ALLOCATE = 14
}