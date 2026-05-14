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

// MMIO REG
#define REG_STATUS 0x0
#define REG_IRQ_STATUS 0x4
#define REG_START_WORK 0x8
#define REG_CONTINUE_WORK 0xc
#define REG_CLEAR_IRQ 0x10
#define REG_PAR0 0x18 // dest_attr_type
#define REG_PAR1 0x20 // allocator_ptr
#define REG_PAR2 0x28 // alloc_region
#define REG_PAR3 0x30 // min_word_size
#define REG_PAR4 0x38 // desired_word_size
#define REG_PAR5 0x40 // freelist_lock_ptr
#define REG_PAR6 0x48 // thread
#define REG_IRQ_PAR0 0x50
#define REG_IRQ_PAR1 0x58
#define REG_IRQ_PAR2 0x60
#define REG_IRQ_PAR3 0x68
#define REG_IRQ_RES0 0x70 // obj_ptr
#define REG_IRQ_RES1 0x78 // actual_plab_size

#define ALLOCATE_IRQ 0x01
#define ATTEMPT_IRQ 0x02
#define LOCK_WAKE_IRQ 0x04
#define PAGE_FAULT_IRQ 0x08
#define COMPLETE_IRQ 0x10
#define ATOMIC_IRQ 0x20

#define HWGC_STATUS_COMPUTING 0x01
#define HWGC_STATUS_WAKE 0x02
#define HWGC_STATUS_IRQ 0x04

#define HWGC_TLB_SIZE 1048576 * 16

typedef struct
{
    vaddr va_page;
    hwaddr pa_page;
} HWGCTLBEntry;

struct HWGC_PARALLOCATE_PARS
{
    int8_t dest_attr_type;
    uint64_t allocator_ptr;
    uint64_t alloc_region;
    uint64_t min_word_size;
    uint64_t desired_word_size;
    uint64_t freelist_lock_ptr;
    uint64_t thread;
};

struct HWGC_IRQ_PARS
{
    // read fault: addr size data
    // write fault: addr size wdata
    uint64_t par0;
    uint64_t par1;
    uint64_t par2;
    uint64_t par3;
    uint64_t obj_ptr;
    uint64_t actual_word_size;
};

enum HWGC_EXEC_STEP
{
    STEP_PAR_ALLOCATE,
    STEP_ALLOCATE_IML,
    STEP_ALLOCATE,
    STEP_ATTEMPT_IRQ,
    STEP_ALLOCATE_IRQ,
    STEP_PAGE_FAULT,
    STEP_LOCK_WAKE,
    STEP_ATOMIC_IRQ,
    STEP_DONE,
};