#include "hwgc.h"

#define TYPE_PCI_HWGC_DEVICE "hwgc"
typedef struct HWGCState HWGCState;
DECLARE_INSTANCE_CHECKER(HWGCState, HWGC, TYPE_PCI_HWGC_DEVICE)

struct HWGCState
{
    PCIDevice pdev;
    MemoryRegion mmio;

    QemuThread thread;
    QemuMutex thr_mutex;
    QemuCond thr_cond;
    bool stop;

    // reg data
    uint32_t status;
    uint32_t irq_status;

    bool cont;
    struct HWGCParameter pars;
    struct HWGCSoftCallParameter softPars;
    uint64_t soft_res;
    enum HWGC_EXEC_STEP current;
    enum HWGC_EXEC_STEP prev;
};

static uint64_t get_device_id(HWGCState *s) { return HWGC_DEVICE_ID; }
static uint64_t get_status(HWGCState *s) { return qatomic_read(&s->status); }
static uint64_t get_int_status(HWGCState *s) { return s->irq_status; }
static uint64_t get_soft_par0(HWGCState *s) { return s->softPars.par0; }
static uint64_t get_soft_par1(HWGCState *s) { return s->softPars.par1; }
static uint64_t get_soft_par2(HWGCState *s) { return s->softPars.par2; }
static uint64_t get_soft_par3(HWGCState *s) { return s->softPars.par3; }

static hwaddr vaddr2hwaddr(uintptr_t vaddr)
{
    CPUState *cpu = qemu_get_cpu(0);
    hwaddr ha = cpu_get_phys_page_debug(cpu, vaddr & TARGET_PAGE_MASK) | (vaddr & ~TARGET_PAGE_MASK);
    return ha;
}
static int accessHWAddr(uintptr_t va, uint64_t *value, int size)
{
    hwaddr ha = vaddr2hwaddr(va);
    if (ha != -1)
    {
        MemTxResult res = address_space_read(&address_space_memory, ha, MEMTXATTRS_UNSPECIFIED, value, 4);
        if (res == MEMTX_OK)
            printf("the data is %lx\n", *value);
        else
            printf("address space read failed\n");
    }
    else
    {
        printf("vaddr to hwaddr is failed\n");
        return -1;
    }
}

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
    if (size != 4 && size != 8)
        return ~0ULL;
    struct
    {
        hwaddr offset;
        unsigned char width; // 4 or 8
        uint64_t (*get)(HWGCState *);
    } regs[] = {
        {REG_DEVICE_ID, 4, get_device_id},
        {REG_STATUS, 4, get_status},
        {REG_INT_STATUS, 4, get_int_status},
        {REG_SOFT_PAR0, 8, get_soft_par0},
        {REG_SOFT_PAR1, 8, get_soft_par1},
        {REG_SOFT_PAR2, 8, get_soft_par2},
        {REG_SOFT_PAR3, 8, get_soft_par3},
    };
    for (int i = 0; i < ARRAY_SIZE(regs); i++)
    {
        if (regs[i].offset == addr)
        {
            if (size != regs[i].width)
                return ~0ULL;
            return regs[i].get(hwgc);
        }
    }
    return ~0ULL;
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
    if (addr >= REG_PAR0 && addr <= REG_PAR13)
    {
        if (qatomic_read(&hwgc->status) & HWGC_STATUS_COMPUTING || size != 8)
            return;
        static const size_t par_offsets[] = {
            offsetof(struct HWGCParameter, chunkSize),                       // REG_PAR0: lo=chunkSize, hi=ageThreshold
            offsetof(struct HWGCParameter, heapRegionBias),                  // REG_PAR1: lo=heapRegionBias, hi=regionAttrShiftBy
            offsetof(struct HWGCParameter, heapRegionShiftBy),               // REG_PAR2: lo=heapRegionShiftBy, hi=logOfHRGrainBytes
            offsetof(struct HWGCParameter, stepperOffset),                   // REG_PAR3
            offsetof(struct HWGCParameter, youngWordsBase),                  // REG_PAR4
            offsetof(struct HWGCParameter, regionAttrBase),                  // REG_PAR5
            offsetof(struct HWGCParameter, plabAllocatorPtr),                // REG_PAR6
            offsetof(struct HWGCParameter, regionAttrBiasedBase),            // REG_PAR7
            offsetof(struct HWGCParameter, heapRegionBiasedBase),            // REG_PAR8
            offsetof(struct HWGCParameter, parScanThreadStatePtr),           // REG_PAR9
            offsetof(struct HWGCParameter, taskQueueBottomAddr),             // REG_PAR10
            offsetof(struct HWGCParameter, taskQueueAgeTopAddr),             // REG_PAR11
            offsetof(struct HWGCParameter, taskQueueElemsBase),              // REG_PAR12
            offsetof(struct HWGCParameter, humogousReclaimCandidateBoolBase) // REG_PAR13
        };
        int idx = addr - REG_PAR0;
        uint8_t *base = (uint8_t *)&hwgc->pars;
        if (idx <= 2)
        {
            uint32_t *lo = (uint32_t *)(base + par_offsets[idx]);
            uint32_t *hi = lo + 1;
            *lo = (uint32_t)val;
            *hi = (uint32_t)(val >> 32);
        }
        else
            *(uint64_t *)(base + par_offsets[idx]) = val;
    }

    if (addr == REG_START_WORK)
    {
        hwgc->current = STEP_FETCH;
        qatomic_or(&hwgc->status, HWGC_STATUS_COMPUTING);
        qemu_cond_signal(&hwgc->thr_cond);
    }

    if (addr == REG_CONTINUE_WORK)
    {
        hwgc->cont = true;
        qemu_cond_signal(&hwgc->thr_cond);
    }

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

static void do_hwgc_work(void *opaque, struct HWGCParameter)
{
    static uint64_t task;

    HWGCState *hwgc = opaque;

    if (hwgc->current == STEP_FETCH)
    {
        uint64_t localBot, ageTop;
        accessHWAddr(hwgc->pars.taskQueueBottomAddr, &localBot, 4);
        accessHWAddr(hwgc->pars.taskQueueAgeTopAddr, &ageTop, 4);
        printf("localBot %lx AgeTop %lx\n", localBot, ageTop);
        hwgc->current = STEP_DONE;
    }
}

static void *hwgc_work_thread(void *opaque)
{
    HWGCState *hwgc = opaque;

    while (1)
    {
        printf("do hwgc work\n");
        struct HWGCParameter hwgc_par;
        qemu_mutex_lock(&hwgc->thr_mutex);
        while ((qatomic_read(&hwgc->status) & HWGC_STATUS_COMPUTING) == 0 && !hwgc->stop)
            qemu_cond_wait(&hwgc->thr_cond, &hwgc->thr_mutex);

        if (hwgc->stop)
        {
            qemu_mutex_unlock(&hwgc->thr_mutex);
            break;
        }

        hwgc_par = hwgc->pars;
        qemu_mutex_unlock(&hwgc->thr_mutex);

        while (1)
        {
            do_hwgc_work(hwgc, hwgc_par);
            if (hwgc->current == STEP_DONE)
                break;
            if (hwgc->current == STEP_ALLOCATE_SLOW)
            {
                qemu_mutex_lock(&hwgc->thr_mutex);
                while (!hwgc->cont && !hwgc->stop)
                    qemu_cond_wait(&hwgc->thr_cond, &hwgc->thr_mutex);
                printf("wait complete\n");
                hwgc->cont = false;
                hwgc->current = STEP_AOP;
                qemu_mutex_unlock(&hwgc->thr_mutex);
                continue;
            }
        }

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
    hwgc->stop = true;
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
    k->revision = 0x1;                             // 设备的修订版本号
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