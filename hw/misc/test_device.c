/*
 * QEMU PCI demo device: xor-tlbdev
 *
 * Model:
 *   - Guest driver passes user virtual addresses of a, b, and out.
 *   - Device has a tiny private TLB: VA-page -> guest physical page.
 *   - On TLB miss, device raises IRQ_TLB_MISS and exposes miss_va/miss_access.
 *   - Linux driver pins/translates the user page and writes TLB fill registers.
 *   - Device then continues.
 *
 * Work pipeline:
 *   effective cycle 1: read u64 a and b from translated addresses
 *   effective cycle 2: compute a ^ b
 *   effective cycle 3: write u64 result
 *
 * Stalls:
 *   TLB misses pause the timer/worker. After the driver fills the TLB and writes
 *   CMD_CONTINUE, the device retries the same stage on the next tick.
 *
 * Integration:
 *   Put this file under hw/misc/xor_tlbdev.c, add it to hw/misc/meson.build:
 *     system_ss.add(when: 'CONFIG_TEST_DEVICE', if_true: files('xor_tlbdev.c'))
 *
 * Launch example:
 *   qemu-system-x86_64 ... -device xor-tlbdev
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qemu/timer.h"
#include "qemu/main-loop.h"
#include "qemu/module.h"
#include "qom/object.h"
#include "hw/pci/pci.h"
#include "hw/pci/msi.h"
#include "exec/target_page.h"
#include "system/address-spaces.h"

#define TYPE_XOR_TLBDEV "xor-tlbdev"

typedef struct XorTLBDevState XorTLBDevState;
DECLARE_INSTANCE_CHECKER(XorTLBDevState, XOR_TLBDEV, TYPE_XOR_TLBDEV)

/* PCI IDs: QEMU test vendor 0x1234 + demo device id. Match in Linux driver. */
#define XOR_VENDOR_ID PCI_VENDOR_ID_QEMU // QEMU预留的测试厂商ID
#define XOR_DEVICE_ID 0x0420

/* MMIO register map */
#define REG_STATUS 0x00     // rw32: IRQ_EN bit writable; status bits readable
#define REG_IRQ_STATUS 0x04 // ro32
#define REG_IRQ_CLEAR 0x08  // wo32
#define REG_CMD 0x0c        // wo32

#define REG_A_VA 0x10   // rw64
#define REG_B_VA 0x18   // rw64
#define REG_OUT_VA 0x20 // rw64

#define REG_MISS_VA 0x28     // ro64
#define REG_MISS_ACCESS 0x30 // ro32: 1=read, 2=write
#define REG_TLB_FILL_VA 0x38 // wo64: page-aligned VA page
#define REG_TLB_FILL_PA 0x40 // wo64: page-aligned guest physical page

#define REG_PERIOD_NS 0x48 // rw64: simulated device clock period
#define REG_STAGE 0x50     // ro32

/* Command bits */
#define CMD_START 0x01
#define CMD_CONTINUE 0x02
#define CMD_RESET 0x04

/* Status bits */
#define ST_BUSY 0x00000001
#define ST_DONE 0x00000002
#define ST_WAIT_TLB 0x00000004
#define ST_ERROR 0x00000008
#define ST_IRQ_EN 0x00000010

/* IRQ bits */
#define IRQ_TLB_MISS 0x00000001
#define IRQ_DONE 0x00000002
#define IRQ_ERROR 0x00000004

/* Device access type reported to driver */
#define ACCESS_READ 1
#define ACCESS_WRITE 2

/* Local page constants (independent of target arch macros) */
#define XOR_PAGE_SHIFT 14 // linux里这里是14
#define XOR_PAGE_SIZE (1ULL << XOR_PAGE_SHIFT)
#define XOR_PAGE_MASK (XOR_PAGE_SIZE - 1)

#define XOR_TLB_SIZE 16

typedef struct XorTLBEntry
{
    bool valid;
    uint64_t va_page;
    hwaddr pa_page;
} XorTLBEntry;

enum XorStage
{
    STAGE_IDLE = 0,
    STAGE_FETCH = 1,
    STAGE_EXEC = 2,
    STAGE_WRITEBACK = 3,
    STAGE_DONE = 4,
};

struct XorTLBDevState
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
    enum XorStage stage;

    uint64_t a_va;
    uint64_t b_va;
    uint64_t out_va;

    uint64_t miss_va;
    uint32_t miss_access;

    uint64_t fill_va;
    hwaddr fill_pa;

    uint64_t a_val;
    uint64_t b_val;
    uint64_t result;

    XorTLBEntry tlb[XOR_TLB_SIZE];
};

static uint32_t xor_tlb_hash(uint64_t va_page)
{
    return (va_page >> XOR_PAGE_SHIFT) & (XOR_TLB_SIZE - 1);
}

static void xor_tlb_flush(XorTLBDevState *s)
{
    memset(s->tlb, 0, sizeof(s->tlb));
}

static void xor_tlb_insert_locked(XorTLBDevState *s,
                                  uint64_t va,
                                  hwaddr pa)
{
    uint64_t va_page;
    hwaddr pa_page;
    uint32_t idx;

    va_page = va & ~XOR_PAGE_MASK;
    pa_page = pa & ~(hwaddr)XOR_PAGE_MASK;
    idx = xor_tlb_hash(va_page);

    s->tlb[idx].valid = true;
    s->tlb[idx].va_page = va_page;
    s->tlb[idx].pa_page = pa_page;

    printf("[xor] TLB insert: idx=%u va_page=0x%016" PRIx64
           " pa_page=0x%016" HWADDR_PRIx "\n",
           idx, va_page, pa_page);
}

static bool xor_tlb_lookup_locked(XorTLBDevState *s,
                                  uint64_t va,
                                  hwaddr *pa)
{
    uint64_t va_page;
    uint64_t offset;
    uint32_t idx;
    XorTLBEntry *e;

    va_page = va & ~XOR_PAGE_MASK;
    offset = va & XOR_PAGE_MASK;
    idx = xor_tlb_hash(va_page);
    e = &s->tlb[idx];

    if (!e->valid || e->va_page != va_page)
        return false;

    *pa = e->pa_page | offset;
    return true;
}

static bool xor_msi_enabled(XorTLBDevState *s)
{
    return msi_enabled(&s->pdev);
}

// Called from the device worker thread to raise irq. Take BQL around QEMU interrupt injection.
static void xor_raise_irq_from_worker(XorTLBDevState *s, uint32_t bits)
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

//  Called from Software by driver to lower irq, normally already in the QEMU IO/vCPU context.
static void xor_lower_irq_from_mmio(XorTLBDevState *s, uint32_t bits)
{
    qatomic_and(&s->irq_status, ~bits);

    if (!qatomic_read(&s->irq_status) && !xor_msi_enabled(s))
        pci_set_irq(&s->pdev, 0);
}

static void xor_arm_timer(XorTLBDevState *s)
{
    timer_mod_ns(s->tick_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + s->period_ns);
}

static void xor_pause_for_tlb_miss(XorTLBDevState *s, uint64_t va, uint32_t access)
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
static bool xor_translate(XorTLBDevState *s, uint64_t va, uint32_t access, hwaddr *pa)
{
    bool hit;

    qemu_mutex_lock(&s->lock);
    hit = xor_tlb_lookup_locked(s, va, pa);
    qemu_mutex_unlock(&s->lock);

    if (!hit)
    {
        printf("translate not hit\n");
        xor_pause_for_tlb_miss(s, va, access);
        return false;
    }

    return true;
}

static bool xor_access_u64(XorTLBDevState *s, uint64_t va, uint64_t *val, bool write)
{
    hwaddr pa;
    MemTxResult tx;

    // demo check va必须8B对齐 且访问必须在一页内
    if ((va & 7) != 0 || ((va & (XOR_PAGE_SIZE - 1)) > XOR_PAGE_SIZE - sizeof(uint64_t)))
    {
        printf("va address error\n");
        qemu_mutex_lock(&s->lock);
        qatomic_or(&s->status, ST_ERROR);
        s->timer_running = false;
        qemu_mutex_unlock(&s->lock);
        xor_raise_irq_from_worker(s, IRQ_ERROR);
        return false;
    }

    if (!xor_translate(s, va, write ? ACCESS_WRITE : ACCESS_READ, &pa))
        return false;

    bql_lock();
    if (write)
        tx = address_space_write(&address_space_memory, pa, MEMTXATTRS_UNSPECIFIED, (uint8_t *)val, sizeof(*val));
    else
        tx = address_space_read(&address_space_memory, pa, MEMTXATTRS_UNSPECIFIED, (uint8_t *)val, sizeof(*val));
    bql_unlock();

    if (tx != MEMTX_OK)
    {
        printf("memory access error\n");
        qemu_mutex_lock(&s->lock);
        qatomic_or(&s->status, ST_ERROR);
        s->timer_running = false;
        qemu_mutex_unlock(&s->lock);
        xor_raise_irq_from_worker(s, IRQ_ERROR);
        return false;
    }

    return true;
}

static void xor_step_one_effective_cycle(XorTLBDevState *s)
{
    enum XorStage stage;
    uint64_t a_va, b_va, out_va;
    uint64_t a, b, r;

    qemu_mutex_lock(&s->lock);
    if (!(qatomic_read(&s->status) & ST_BUSY) || s->timer_running == false)
    {
        qemu_mutex_unlock(&s->lock);
        return;
    }

    stage = s->stage;
    a_va = s->a_va;
    b_va = s->b_va;
    out_va = s->out_va;
    a = s->a_val;
    b = s->b_val;
    r = s->result;
    qemu_mutex_unlock(&s->lock);

    switch (stage)
    {
    case STAGE_FETCH:
        // Effective cycle 1: read a and b. If either VA misses, this cycle stalls.
        if (!xor_access_u64(s, a_va, &a, false))
            return;
        if (!xor_access_u64(s, b_va, &b, false))
            return;

        qemu_mutex_lock(&s->lock);
        s->a_val = a;
        s->b_val = b;
        s->stage = STAGE_EXEC;
        qemu_mutex_unlock(&s->lock);
        printf("a %lx b %lx\n", a, b);
        xor_arm_timer(s);
        break;

    case STAGE_EXEC:
        // Effective cycle 2:   calculate a ^ b.
        r = a ^ b;
        qemu_mutex_lock(&s->lock);
        s->result = r;
        s->stage = STAGE_WRITEBACK;
        qemu_mutex_unlock(&s->lock);
        printf("result %lx\n", r);
        xor_arm_timer(s); // 过period_ns后,timer callback 唤醒 worker
        break;

    case STAGE_WRITEBACK:
        // Effective cycle 3:   write result. If out VA misses, this cycle stalls.
        if (!xor_access_u64(s, out_va, &r, true))
            return;

        qemu_mutex_lock(&s->lock);
        s->stage = STAGE_DONE;
        s->timer_running = false;
        qatomic_and(&s->status, ~ST_BUSY);
        qatomic_or(&s->status, ST_DONE);
        qemu_mutex_unlock(&s->lock);

        xor_raise_irq_from_worker(s, IRQ_DONE);
        break;

    default:
        break;
    }
}

static void xor_tick_timer_cb(void *opaque)
{
    XorTLBDevState *s = opaque;

    qemu_mutex_lock(&s->lock);

    // tick_pending表示有一次的tick需要处理
    if (!s->thread_stop && s->timer_running)
    {
        s->tick_pending = true;
        qemu_cond_signal(&s->cond);
    }

    qemu_mutex_unlock(&s->lock);
}

static void *xor_worker_thread(void *opaque)
{
    XorTLBDevState *s = opaque;

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

        xor_step_one_effective_cycle(s);
    }

    return NULL;
}

static void xor_reset_device_locked(XorTLBDevState *s)
{
    s->timer_running = false;
    s->tick_pending = false;

    qatomic_set(&s->status, qatomic_read(&s->status) & ST_IRQ_EN);
    qatomic_set(&s->irq_status, 0);

    s->stage = STAGE_IDLE;
    s->a_va = 0;
    s->b_va = 0;
    s->out_va = 0;
    s->miss_va = 0;
    s->miss_access = 0;
    s->fill_va = 0;
    s->fill_pa = 0;
    s->a_val = 0;
    s->b_val = 0;
    s->result = 0;

    xor_tlb_flush(s);
}

static void xor_start_locked(XorTLBDevState *s)
{
    if (qatomic_read(&s->status) & ST_BUSY)
        return;

    qatomic_and(&s->status, ST_IRQ_EN);
    qatomic_or(&s->status, ST_BUSY);

    qatomic_set(&s->irq_status, 0);
    s->miss_va = 0;
    s->miss_access = 0;
    s->a_val = 0;
    s->b_val = 0;
    s->result = 0;
    s->stage = STAGE_FETCH;

    xor_tlb_flush(s);

    s->timer_running = true;
    s->tick_pending = false;
    xor_arm_timer(s);
}

static uint64_t xor_mmio_read(void *opaque, hwaddr addr, unsigned size)
{
    XorTLBDevState *s = opaque;
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
    case REG_A_VA:
        ret = s->a_va;
        break;
    case REG_B_VA:
        ret = s->b_va;
        break;
    case REG_OUT_VA:
        ret = s->out_va;
        break;
    case REG_MISS_VA:
        ret = s->miss_va;
        break;
    case REG_MISS_ACCESS:
        ret = s->miss_access;
        break;
    case REG_PERIOD_NS:
        ret = s->period_ns;
        break;
    case REG_STAGE:
        ret = s->stage;
        break;
    default:
        printf("[xor] unknown MMIO read: addr=0x%" HWADDR_PRIx
               ", size=%u\n",
               addr, size);
        ret = ~0ULL;
        break;
    }

    printf("[xor] MMIO read: addr=0x%" HWADDR_PRIx
           ", size=%u, value=0x%016" PRIx64 "\n",
           addr, size, ret);

    qemu_mutex_unlock(&s->lock);
    return ret;
}

static void xor_mmio_write(void *opaque, hwaddr addr,
                           uint64_t val, unsigned size)
{
    XorTLBDevState *s = opaque;
    bool do_continue = false;

    printf("[xor] MMIO write: addr=0x%" HWADDR_PRIx
           ", size=%u, value=0x%016" PRIx64 "\n",
           addr, size, val);

    if (addr == REG_IRQ_CLEAR && size == 4)
    {
        printf("[xor] IRQ clear: value=0x%08" PRIx64 "\n",
               val & UINT64_C(0xffffffff));

        xor_lower_irq_from_mmio(s, val);
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
                printf("[xor] enable IRQ\n");
                qatomic_or(&s->status, ST_IRQ_EN);
            }
            else
            {
                printf("[xor] disable IRQ\n");
                qatomic_and(&s->status, ~ST_IRQ_EN);
            }
        }
        else
        {
            printf("[xor] invalid STATUS size: %u\n", size);
        }
        break;

    case REG_CMD:
        if (size == 4)
        {
            if (val & CMD_RESET)
            {
                printf("[xor] CMD_RESET\n");
                xor_reset_device_locked(s);
            }

            if (val & CMD_START)
            {
                printf("[xor] CMD_START: "
                       "a_va=0x%016" PRIx64 ", "
                       "b_va=0x%016" PRIx64 ", "
                       "out_va=0x%016" PRIx64 "\n",
                       s->a_va, s->b_va, s->out_va);

                xor_start_locked(s);
            }

            if (val & CMD_CONTINUE)
            {
                printf("[xor] CMD_CONTINUE: "
                       "fill_va=0x%016" PRIx64 ", "
                       "fill_pa=0x%016" PRIx64 "\n",
                       s->fill_va, s->fill_pa);

                xor_tlb_insert_locked(s, s->fill_va, s->fill_pa);
                qatomic_and(&s->status, ~ST_WAIT_TLB);

                s->miss_access = 0;
                s->timer_running = true;
                s->tick_pending = false;
                do_continue = true;
            }
        }
        else
        {
            printf("[xor] invalid CMD size: %u\n", size);
        }
        break;

    case REG_A_VA:
        if (size == 8)
        {
            printf("[xor] set A_VA: 0x%016" PRIx64 "\n", val);
            s->a_va = val;
        }
        break;

    case REG_B_VA:
        if (size == 8)
        {
            printf("[xor] set B_VA: 0x%016" PRIx64 "\n", val);
            s->b_va = val;
        }
        break;

    case REG_OUT_VA:
        if (size == 8)
        {
            printf("[xor] set OUT_VA: 0x%016" PRIx64 "\n", val);
            s->out_va = val;
        }
        break;

    case REG_TLB_FILL_VA:
        if (size == 8)
        {
            printf("[xor] set TLB_FILL_VA: 0x%016" PRIx64 "\n", val);
            s->fill_va = val;
        }
        break;

    case REG_TLB_FILL_PA:
        if (size == 8)
        {
            printf("[xor] set TLB_FILL_PA: 0x%016" PRIx64 "\n", val);
            s->fill_pa = val;
        }
        break;

    case REG_PERIOD_NS:
        if (size == 8 && val >= 1000)
        {
            printf("[xor] set PERIOD_NS: %" PRIu64 "\n", val);
            s->period_ns = val;
        }
        else
        {
            printf("[xor] invalid PERIOD_NS: value=%" PRIu64
                   ", size=%u\n",
                   val, size);
        }
        break;

    default:
        printf("[xor] unknown MMIO write: addr=0x%" HWADDR_PRIx
               ", size=%u, value=0x%016" PRIx64 "\n",
               addr, size, val);
        break;
    }

    qemu_mutex_unlock(&s->lock);

    if (do_continue)
    {
        printf("[xor] arm timer, period=%" PRIu64 " ns\n",
               s->period_ns);
        xor_arm_timer(s);
    }
}

static const MemoryRegionOps xor_mmio_ops = {
    .read = xor_mmio_read,
    .write = xor_mmio_write,
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

static void xor_tlbdev_realize(PCIDevice *pdev, Error **errp)
{
    XorTLBDevState *s = XOR_TLBDEV(pdev);
    uint8_t *pci_conf = pdev->config;

    pci_config_set_interrupt_pin(pci_conf, 1);

    if (msi_init(pdev, 0, 1, true, false, errp))
        return;

    qemu_mutex_init(&s->lock);
    qemu_cond_init(&s->cond);

    s->period_ns = 1000000; /* default: 1 ms per simulated hardware cycle */
    s->thread_stop = false;
    s->timer_running = false;
    s->tick_pending = false;
    s->stage = STAGE_IDLE;
    xor_tlb_flush(s);

    s->tick_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, xor_tick_timer_cb, s);

    qemu_thread_create(&s->worker, "xor-tlbdev",
                       xor_worker_thread, s, QEMU_THREAD_JOINABLE);

    memory_region_init_io(&s->mmio, OBJECT(s), &xor_mmio_ops, s,
                          "xor-tlbdev-mmio", 4 * KiB);
    pci_register_bar(pdev, 0, PCI_BASE_ADDRESS_SPACE_MEMORY, &s->mmio);
}

static void xor_tlbdev_exit(PCIDevice *pdev)
{
    XorTLBDevState *s = XOR_TLBDEV(pdev);

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

static void xor_tlbdev_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

    k->realize = xor_tlbdev_realize;
    k->exit = xor_tlbdev_exit;
    k->vendor_id = XOR_VENDOR_ID;
    k->device_id = XOR_DEVICE_ID;
    k->revision = 0x01;
    k->class_id = PCI_CLASS_OTHERS;

    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

static const TypeInfo xor_tlbdev_info = {
    .name = TYPE_XOR_TLBDEV,
    .parent = TYPE_PCI_DEVICE,
    .instance_size = sizeof(XorTLBDevState),
    .class_init = xor_tlbdev_class_init,
    .interfaces = (const InterfaceInfo[]){
        {INTERFACE_CONVENTIONAL_PCI_DEVICE},
        {},
    },
};

static void xor_tlbdev_register_types(void)
{
    type_register_static(&xor_tlbdev_info);
}

type_init(xor_tlbdev_register_types)