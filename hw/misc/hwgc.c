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

#define TYPE_PCI_HWGC_DEVICE "hwgc"
typedef struct HWGCState HWGCState;
DECLARE_INSTANCE_CHECKER(HWGCState, HWGC, TYPE_PCI_HWGC_DEVICE)

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
#define REG_START_WORK 0x80

#define ALLOC_SLOW_IRQ 0x00000001
#define ENQUEUE_FAILED_IRQ 0x00000100
#define GC_COMPLETE_IRQ 0x00010000

#define HWGC_DEVICE_ID 0x20020420

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
    uint64_t taskQueueAgeTopAddr;
    uint64_t taskQueueElemsBase;
    uint64_t humogousReclaimCandidateBoolBase;
};

struct HWGCState
{
    PCIDevice pdev;
    MemoryRegion mmio;

    QemuThread thread;
    QemuMutex thr_mutex;
    QemuCond thr_cond;
    bool stopping;

// reg data
#define HWGC_STATUS_COMPUTING 0x01
#define HWGC_STATUS_IRQ 0x80
    uint32_t status;
    uint32_t irq_status;

    struct HWGCParameter pars;
};

// PCI/PCIe 支持两种中断机制
//      MSI: 边沿出发 不需要维持状态 发一次会自动无效掉
//      Legacy INTx: 电平触发 需要手动拉低 hwgc_lower_irq
// 检查是否启用 msi function
static bool hwgc_msi_enabled(HWGCState *hwgc)
{
    return msi_enabled(&hwgc->pdev);
}

static void hwgc_raise_irq(HWGCState *hwgc, uint32_t val)
{
    hwgc->irq_status |= val; // 记录当前有哪些中断 pending
    if (hwgc->irq_status)
    {
        if (hwgc_msi_enabled(hwgc))
        {
            printf("raise msi\n");
            msi_notify(&hwgc->pdev, 0);
        }
        else
        {
            printf("raise legacy intx\n");
            pci_set_irq(&hwgc->pdev, 1);
        }
    }
}

static void hwgc_lower_irq(HWGCState *hwgc, uint32_t val)
{
    hwgc->irq_status &= ~val;

    if (!hwgc->irq_status && !hwgc_msi_enabled(hwgc))
        pci_set_irq(&hwgc->pdev, 0);
}

static uint64_t hwgc_mmio_read(void *opaque, hwaddr addr, unsigned size)
{
    HWGCState *hwgc = opaque;
    uint64_t val = ~0ULL;
    if (addr < REG_CLEAR_IRQ && size != 4)
        return val;

    if (addr >= REG_CLEAR_IRQ)
        return val;

    switch (addr)
    {
    case REG_DEVICE_ID:
        val = HWGC_DEVICE_ID;
        break;
    case REG_STATUS:
        val = qatomic_read(&hwgc->status);
        break;
    case REG_INT_STATUS:
        val = hwgc->irq_status;
        break;
    default:
        break;
    }
    return val;
}

static void hwgc_mmio_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    HWGCState *hwgc = opaque;
    printf("%lx %x %lx\n", addr, size, val);
    if (addr < REG_CLEAR_IRQ && size != 4)
        return;
    else if (addr >= REG_PAR0 && size != 8 && size != 4)
        return;

    if (addr <= REG_CLEAR_IRQ)
    {
        switch (addr)
        {
        case REG_STATUS:
            if (val & HWGC_STATUS_IRQ)
            {
                qatomic_or(&hwgc->status, HWGC_STATUS_IRQ);
                smp_mb__after_rmw();
            }
            else
                qatomic_and(&hwgc->status, ~HWGC_STATUS_IRQ);
            break;
        case REG_CLEAR_IRQ:
            hwgc_lower_irq(hwgc, val);
            break;
        }
        return;
    }

    if (qatomic_read(&hwgc->status) & HWGC_STATUS_COMPUTING)
        return;

    qemu_mutex_lock(&hwgc->thr_mutex);

    /* 处理参数寄存器 REG_PAR0 ~ REG_PAR13 */
    if (addr >= REG_PAR0 && addr <= REG_PAR13)
    {
        uint64_t *dst = NULL;
        switch (addr)
        {
        case REG_PAR0:
            dst = (uint64_t *)&hwgc->pars.chunkSize;
            break;
        case REG_PAR1:
            dst = (uint64_t *)&hwgc->pars.heapRegionBias;
            break;
        case REG_PAR2:
            dst = (uint64_t *)&hwgc->pars.heapRegionShiftBy;
            break;
        case REG_PAR3:
            dst = &hwgc->pars.stepperOffset;
            break;
        case REG_PAR4:
            dst = &hwgc->pars.youngWordsBase;
            break;
        case REG_PAR5:
            dst = &hwgc->pars.regionAttrBase;
            break;
        case REG_PAR6:
            dst = &hwgc->pars.plabAllocatorPtr;
            break;
        case REG_PAR7:
            dst = &hwgc->pars.regionAttrBiasedBase;
            break;
        case REG_PAR8:
            dst = &hwgc->pars.heapRegionBiasedBase;
            break;
        case REG_PAR9:
            dst = &hwgc->pars.parScanThreadStatePtr;
            break;
        case REG_PAR10:
            dst = &hwgc->pars.taskQueueBottomAddr;
            break;
        case REG_PAR11:
            dst = &hwgc->pars.taskQueueAgeTopAddr;
            break;
        case REG_PAR12:
            dst = &hwgc->pars.taskQueueElemsBase;
            break;
        case REG_PAR13:
            dst = &hwgc->pars.humogousReclaimCandidateBoolBase;
            break;
        default:
            goto unlock_out;
        }

        if (addr <= REG_PAR2)
        {
            *(uint32_t *)dst = (uint32_t)val;
            *(uint32_t *)(dst + 1) = (uint32_t)(val >> 32);
        }
        else
            *dst = val;
    }
    else if (addr == REG_START_WORK)
    {
        qatomic_or(&hwgc->status, HWGC_STATUS_COMPUTING);
        qemu_cond_signal(&hwgc->thr_cond);
    }

unlock_out:
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

static void *hwgc_work_thread(void *opaque)
{
    HWGCState *hwgc = opaque;

    while (1)
    {
        struct HWGCParameter hwgc_par;
        qemu_mutex_lock(&hwgc->thr_mutex);
        while ((qatomic_read(&hwgc->status) & HWGC_STATUS_COMPUTING) == 0 && !hwgc->stopping)
            qemu_cond_wait(&hwgc->thr_cond, &hwgc->thr_mutex);

        // 如果hwgc设备被关闭 那么break
        if (hwgc->stopping)
        {
            qemu_mutex_unlock(&hwgc->thr_mutex);
            break;
        }

        hwgc_par = hwgc->pars;
        qemu_mutex_unlock(&hwgc->thr_mutex);

        // 计算阶乘

        // 计算完成
        qatomic_and(&hwgc->status, ~HWGC_STATUS_COMPUTING);
        smp_mb__after_rmw(); // 确保后续的read hwgc->status 不会排到该write之前
        // 允许计算完成后 发起中断
        if (qatomic_read(&hwgc->status) & HWGC_STATUS_IRQ)
        {
            bql_lock(); // 确保可以在非主线程中调用主线程函数(pci_set_irq, msi_notify)
            hwgc_raise_irq(hwgc, GC_COMPLETE_IRQ);
            bql_unlock();
        }
    }
    return NULL;
}

static void pci_hwgc_realize(PCIDevice *pdev, Error **errp)
{
    HWGCState *hwgc = HWGC(pdev);
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

    // 设置 设备停止
    qemu_mutex_lock(&hwgc->thr_mutex);
    hwgc->stopping = true;
    qemu_mutex_unlock(&hwgc->thr_mutex);
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
    k->revision = 0x0;                             // 设备的修订版本号
    k->class_id = PCI_CLASS_OTHERS;                // 设备类别 PCI_CLASS_OTHERS属于未分类类别
    set_bit(DEVICE_CATEGORY_MISC, dc->categories); // 分组 在MISC组内显示 qemu -device help
}

static const TypeInfo hwgc_types[] = {
    {
        .name = TYPE_PCI_HWGC_DEVICE,
        .parent = TYPE_PCI_DEVICE,
        .instance_size = sizeof(HWGCState),
        .instance_init = hwgc_instance_init,
        .class_init = hwgc_class_init,
        .interfaces = (const InterfaceInfo[]){
            {INTERFACE_CONVENTIONAL_PCI_DEVICE},
            {},
        },
    }};

DEFINE_TYPES(hwgc_types)