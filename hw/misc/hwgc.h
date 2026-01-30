#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/units.h"
#include "hw/pci/pci.h"
#include "hw/pci/msi.h"
#include "qemu/timer.h"
#include "qom/object.h"
#include "qemu/main-loop.h" /* iothread mutex */
#include "qemu/module.h"
#include "qapi/visitor.h"
#include "hw/core/cpu.h"
#include "exec/target_page.h"
#include "system/address-spaces.h"

// #define DEBUG_ENABLE 1

// MMIO REG
#define REG_DEVICE_ID 0x0
#define REG_STATUS 0x4
#define REG_INT_STATUS 0x8
#define REG_CLEAR_IRQ 0xc
#define REG_PAR0 0x10
#define REG_PAR1 0x18
#define REG_PAR2 0x20
#define REG_PAR3 0x28
#define REG_PAR4 0x30
#define REG_PAR5 0x38
#define REG_PAR6 0x40
#define REG_PAR7 0x48
#define REG_PAR8 0x50
#define REG_PAR9 0x58
#define REG_PAR10 0x60
#define REG_PAR11 0x68
#define REG_PAR12 0x70
#define REG_PAR13 0x78
#define REG_PAR14 0x80
#define REG_PAR15 0x88
#define REG_PAR16 0x90
#define REG_PAR17 0x98
#define REG_PAR18 0xa0
#define REG_PAR19 0xa8
#define REG_PAR20 0xb0
#define REG_PAR21 0xb8
#define REG_START_WORK 0xc0
#define REG_CONTINUE_WORK 0xc4
#define REG_SOFT_RES 0xc8
#define REG_SOFT_PAR0 0xd0
#define REG_SOFT_PAR1 0xd8
#define REG_SOFT_PAR2 0xe0
#define REG_SOFT_PAR3 0xe8

#define ALLOC_SLOW_IRQ 0x00000001
#define ENQUEUE_FAILED_IRQ 0x00000010
#define PAGE_FAULT_IRQ 0x00000100
#define COMPLETE_IRQ 0x00001000
#define DEBUG_IRQ 0x00010000

#define HWGC_DEVICE_ID 0x20020420

#define HWGC_STATUS_COMPUTING 0x01
#define HWGC_STATUS_WAKE 0x02
#define HWGC_STATUS_IRQ 0x04

#define HWGC_TLB_SIZE 1048576 * 16

typedef struct
{
    vaddr va_page;
    hwaddr pa_page;
} HWGCTLBEntry;

struct HWGCParameter
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
    uint64_t parScanThreadStatePtr;
    uint64_t taskQueueBottomAddr;
    uint64_t taskQueueElemsBase;
    uint64_t humogousReclaimCandidateBoolBase;
    uint64_t cardTablePtr;
    uint64_t g1h;
    uint64_t intArrayKlassObj;
    uint64_t objectKlass;
    uint64_t lockPtr;
    uint64_t thread;
    uint64_t dummyRegion;
    uint64_t numaPtr;
    uint64_t compressedOopBase;
    uint64_t compressedKlassPointerBase;
    uint8_t compressedOopShift;
    uint8_t compressedKlassPointerShift;
    uint8_t useCompressedOops;
    uint8_t useCompressedKlassPointers;
};

struct HWGCSoftRelated
{
    uint64_t par0;
    uint64_t par1;
    uint64_t par2;
    uint64_t par3;
    uint64_t res;
};

enum HWGC_EXEC_STEP
{
    STEP_FETCH = 0,

    STEP_DISPATCH,

    STEP_PARTIAL_ARRAY,

    STEP_COMMON_OOP,
    STEP_Copy2Survivor,

    STEP_ALLOC,
    STEP_ALLOCATE_DIRECT,
    STEP_ALLOCATE_DURING_GC,
    STEP_ATTEMPT_ALLOC,
    STEP_NEW_GC_ALLOC,
    STEP_ALLOC_FREE_REGION,
    STEP_PAR_ALLOCATE_IML,
    STEP_PAR_ALLOCATE,

    STEP_COPY,

    STEP_TRACE,
    STEP_TRACE_PLUS,
    STEP_TRACE_DEC,

    STEP_DO_OOP_WORK,
    STEP_AOP,

    STEP_DEBUG,
    STEP_PAGE_FAULT,

    STEP_DONE,
};

#define TRY_R(next, addr, buf, sz, msg, id)                                            \
    do                                                                                 \
    {                                                                                  \
        tag = safeAccessHWAddr(hwgc, (addr), (buf), (sz), (msg), false, (id), (next)); \
        if (!tag)                                                                      \
            return;                                                                    \
        hwgc->sub_state = (next);                                                      \
    } while (0)

#define TRY_W(next, addr, buf, sz, msg, id)                                           \
    do                                                                                \
    {                                                                                 \
        tag = safeAccessHWAddr(hwgc, (addr), (buf), (sz), (msg), true, (id), (next)); \
        if (!tag)                                                                     \
            return;                                                                   \
        hwgc->sub_state = (next);                                                     \
    } while (0)