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
#define REG_PAR0 0x18 // alloc_region
#define REG_PAR1 0x20 // min_word_size
#define REG_PAR2 0x28 // desired_word_size
#define REG_IRQ_PAR0 0x30
#define REG_IRQ_PAR1 0x38
#define REG_IRQ_PAR2 0x40
#define REG_IRQ_PAR3 0x48
#define REG_IRQ_RES0 0x50 // actual_plab_size
#define REG_IRQ_RES1 0x58 // obj_ptr

#define ATOMIC_IRQ 0x0001
#define PAGE_FAULT_IRQ 0x0010
#define COMPLETE_IRQ 0x0100

#define HWGC_STATUS_COMPUTING 0x01
#define HWGC_STATUS_WAKE 0x02
#define HWGC_STATUS_IRQ 0x04

struct HWGC_PARALLOCATE_PARS
{
    uint64_t alloc_region;
    uint64_t min_word_size;
    uint64_t desired_word_size;
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
    uint64_t actual_plab_size;
};

enum HWGC_EXEC_STEP
{
    STEP_ACCESS_TOP,
    STEP_ACCESS_END,
    STEP_BRANCH,
    STEP_ATOMIC,
    STEP_ATOMIC_RESULT,
    STEP_PAGE_FAULT,
    STEP_DONE,
};