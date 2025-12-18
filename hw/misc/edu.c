/*
 * QEMU educational PCI device
 *
 * Copyright (c) 2012-2015 Jiri Slaby
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

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

#define TYPE_PCI_EDU_DEVICE "edu"
typedef struct EduState EduState;
DECLARE_INSTANCE_CHECKER(EduState, EDU, TYPE_PCI_EDU_DEVICE)

// MMIO REG
#define REG_DEVICE_ID 0x0   // RO
#define REG_INV 0x4         // RW
#define REG_FACT 0x8        // RW
#define REG_STATUS 0x20     // RW
#define REG_INT_STATUS 0x24 // RO
#define REG_RAISE_IRQ 0x60  // WO
#define REG_CLEAR_IRQ 0x64  // WO
#define REG_DMA_SRC 0x80    // RW
#define REG_DMA_DEST 0x88   // RW
#define REG_DMA_CNT 0x90    // RW
#define REG_DMA_CMD 0x98    // RW

#define FACT_IRQ 0x00000001
#define DMA_IRQ 0x00000100

#define DMA_START 0x40000
#define DMA_SIZE 4096

struct EduState
{
    PCIDevice pdev;    // 继承标准PCI设备
    MemoryRegion mmio; // MMIO内存区域

    // 阶乘计算线程相关
    QemuThread thread;
    QemuMutex thr_mutex;
    QemuCond thr_cond;
    bool stopping;

    uint32_t addr4; // 对写入值取反 0x4
    uint32_t fact;  // 阶乘计算输入输出 0x8
#define EDU_STATUS_COMPUTING 0x01
#define EDU_STATUS_IRQFACT 0x80
    uint32_t status; // 状态标志 0x20

    uint32_t irq_status; // 当前哪些irq被触发 0x24

#define EDU_DMA_RUN 0x1
#define EDU_DMA_DIR(cmd) (((cmd) & 0x2) >> 1)
#define EDU_DMA_FROM_PCI 0
#define EDU_DMA_TO_PCI 1
#define EDU_DMA_IRQ 0x4
    struct dma_state
    {
        dma_addr_t src; // 0x80
        dma_addr_t dst; // 0x88
        dma_addr_t cnt; // 0x90
        dma_addr_t cmd; // 0x98
    } dma;
    QEMUTimer dma_timer;    // 模拟DMA延迟
    char dma_buf[DMA_SIZE]; // DMA buffer 0x40000~0x41000
    uint64_t dma_mask;      // DMA地址掩码
};

// PCI/PCIe 支持两种中断机制
//      MSI: 边沿出发 不需要维持状态 发一次会自动无效掉
//      Legacy INTx: 电平触发 需要手动拉低 edu_lower_irq
// 检查是否启用 msi function
static bool edu_msi_enabled(EduState *edu)
{
    return msi_enabled(&edu->pdev);
}

static void edu_raise_irq(EduState *edu, uint32_t val)
{
    edu->irq_status |= val; // 记录当前有哪些中断 pending
    if (edu->irq_status)
    {
        if (edu_msi_enabled(edu))
        {
            printf("raise msi\n");
            msi_notify(&edu->pdev, 0);
        }
        else
        {
            printf("raise legacy intx\n");
            pci_set_irq(&edu->pdev, 1);
        }
    }
}

static void edu_lower_irq(EduState *edu, uint32_t val)
{
    edu->irq_status &= ~val;

    if (!edu->irq_status && !edu_msi_enabled(edu))
        pci_set_irq(&edu->pdev, 0);
}

// 确保 请求的dma区域 xfer 在设备允许的dma区域 dma 内
static void edu_check_range(uint64_t xfer_start, uint64_t xfer_size,
                            uint64_t dma_start, uint64_t dma_size)
{
    uint64_t xfer_end = xfer_start + xfer_size;
    uint64_t dma_end = dma_start + dma_size;

    if (dma_end >= dma_start && xfer_end >= xfer_start &&
        xfer_start >= dma_start && xfer_end <= dma_end)
        return;

    qemu_log_mask(LOG_GUEST_ERROR,
                  "EDU: DMA range 0x%016" PRIx64 "-0x%016" PRIx64
                  " out of bounds (0x%016" PRIx64 "-0x%016" PRIx64 ")!",
                  xfer_start, xfer_end - 1, dma_start, dma_end - 1);
}

static dma_addr_t edu_clamp_addr(const EduState *edu, dma_addr_t addr)
{
    // 地址截断
    dma_addr_t res = addr & edu->dma_mask;

    if (addr != res)
        qemu_log_mask(LOG_GUEST_ERROR,
                      "EDU: clamping DMA 0x%016" PRIx64 " to 0x%016" PRIx64 "!",
                      addr, res);

    return res;
}

// 定时结束后 执行dma
static void edu_dma_timer(void *opaque)
{
    EduState *edu = opaque;
    bool raise_irq = false;

    if (!(edu->dma.cmd & EDU_DMA_RUN))
        return;

    // 设备接收数据
    if (EDU_DMA_DIR(edu->dma.cmd) == EDU_DMA_FROM_PCI)
    {
        uint64_t dst = edu->dma.dst;
        // 检测 [dst, dst + cnt) 在 内存DMA 区域内
        edu_check_range(dst, edu->dma.cnt, DMA_START, DMA_SIZE);
        dst -= DMA_START; // 得到dst相对 内存DMA区域的起始偏移
        pci_dma_read(&edu->pdev, edu_clamp_addr(edu, edu->dma.src),
                     edu->dma_buf + dst, edu->dma.cnt);
    }
    else
    {
        // 设备发送数据
        uint64_t src = edu->dma.src;
        edu_check_range(src, edu->dma.cnt, DMA_START, DMA_SIZE);
        src -= DMA_START;
        pci_dma_write(&edu->pdev, edu_clamp_addr(edu, edu->dma.dst),
                      edu->dma_buf + src, edu->dma.cnt);
    }

    edu->dma.cmd &= ~EDU_DMA_RUN;
    if (edu->dma.cmd & EDU_DMA_IRQ)
        raise_irq = true;

    if (raise_irq)
        edu_raise_irq(edu, DMA_IRQ);
}

static void dma_rw(EduState *edu, bool write, dma_addr_t *val, dma_addr_t *dma,
                   bool timer)
{
    // when dma running, masked all write operations
    if (write && (edu->dma.cmd & EDU_DMA_RUN))
        return;

    if (write)
        *dma = *val;
    else
        *val = *dma;

    // 模拟延迟 在100ms 后调用 &edu->timer
    if (timer)
        timer_mod(&edu->dma_timer, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 100);
}

static uint64_t edu_mmio_read(void *opaque, hwaddr addr, unsigned size)
{
    EduState *edu = opaque;
    uint64_t val = ~0ULL;

    if (addr < REG_DMA_SRC && size != 4)
        return val;

    if (addr >= REG_DMA_SRC && size != 4 && size != 8)
        return val;

    switch (addr)
    {
    case REG_DEVICE_ID:
        val = 0x010000edu;
        break;
    case REG_INV:
        val = edu->addr4;
        break;
    case REG_FACT:
        // Protect concurrent access to the shared variable edu->fact
        // 至于这里为什么不能也用原子读, 猜测原因可能是因为 edu->fact 是共享资源 由其他线程产生 本线程读 存在生产者消费者概念
        //      计算线程: 上锁->计算->解锁      本线程: 上锁->读取->解锁
        // 而对edu->status的修改 只涉及到本线程 而且都是位操作 切换可以瞬间完成
        qemu_mutex_lock(&edu->thr_mutex);
        val = edu->fact;
        qemu_mutex_unlock(&edu->thr_mutex);
        break;
    case 0x20:
        val = qatomic_read(&edu->status);
        break;
    case REG_INT_STATUS:
        val = edu->irq_status;
        break;
    // 通过 dma_rw 函数来完成对 dma 相关数据变量的更新
    case REG_DMA_SRC:
        dma_rw(edu, false, &val, &edu->dma.src, false);
        break;
    case REG_DMA_DEST:
        dma_rw(edu, false, &val, &edu->dma.dst, false);
        break;
    case REG_DMA_CNT:
        dma_rw(edu, false, &val, &edu->dma.cnt, false);
        break;
    case REG_DMA_CMD:
        dma_rw(edu, false, &val, &edu->dma.cmd, false);
        break;
    }

    return val;
}

static void edu_mmio_write(void *opaque, hwaddr addr, uint64_t val,
                           unsigned size)
{
    EduState *edu = opaque;

    if (addr < REG_DMA_SRC && size != 4)
        return;

    if (addr >= REG_DMA_DEST && size != 4 && size != 8)
        return;

    switch (addr)
    {
    case REG_INV:
        edu->addr4 = ~val;
        break;
    case REG_FACT:

        // computing
        if (qatomic_read(&edu->status) & EDU_STATUS_COMPUTING)
            break;

        qemu_mutex_lock(&edu->thr_mutex);
        edu->fact = val;
        qatomic_or(&edu->status, EDU_STATUS_COMPUTING);
        qemu_cond_signal(&edu->thr_cond); // 唤醒后台进程
        qemu_mutex_unlock(&edu->thr_mutex);
        break;
    case REG_STATUS:
        // 控制是否阶乘计算完成时 触发中断
        if (val & EDU_STATUS_IRQFACT)
        {
            qatomic_or(&edu->status, EDU_STATUS_IRQFACT);
            // barrier 确保之后对edu->status的读 不会重排到qatomic_or之前
            smp_mb__after_rmw();
        }
        else
            qatomic_and(&edu->status, ~EDU_STATUS_IRQFACT);
        break;
    case REG_RAISE_IRQ:
        edu_raise_irq(edu, val);
        break;
    case REG_CLEAR_IRQ:
        edu_lower_irq(edu, val);
        break;
    case REG_DMA_SRC:
        dma_rw(edu, true, &val, &edu->dma.src, false);
        break;
    case REG_DMA_DEST:
        dma_rw(edu, true, &val, &edu->dma.dst, false);
        break;
    case REG_DMA_CNT:
        dma_rw(edu, true, &val, &edu->dma.cnt, false);
        break;
    case REG_DMA_CMD:
        // 写该寄存器时 值必须包含EDU_DMA_RUN
        if (!(val & EDU_DMA_RUN))
            break;
        // 写入edu->dma.cmd 并调用 dma.timer
        dma_rw(edu, true, &val, &edu->dma.cmd, true);
        break;
    }
}

static const MemoryRegionOps edu_mmio_ops = {
    .read = edu_mmio_read,
    .write = edu_mmio_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 8,
    },
};

static void *edu_fact_thread(void *opaque)
{
    EduState *edu = opaque;

    while (1)
    {
        uint32_t val, ret = 1;

        qemu_mutex_lock(&edu->thr_mutex);
        // read edu->status, 如果没有在计算状态且edu没有被uninit, 那么等待被唤醒
        while ((qatomic_read(&edu->status) & EDU_STATUS_COMPUTING) == 0 && !edu->stopping)
            qemu_cond_wait(&edu->thr_cond, &edu->thr_mutex);

        // 如果edu设备被关闭 那么break
        if (edu->stopping)
        {
            qemu_mutex_unlock(&edu->thr_mutex);
            break;
        }

        val = edu->fact;
        qemu_mutex_unlock(&edu->thr_mutex); // 读取到了edu->fact 去掉互斥锁

        // 计算阶乘
        while (val > 0)
            ret *= val--;

        // 申请锁 写 edu->fact
        qemu_mutex_lock(&edu->thr_mutex);
        edu->fact = ret;
        printf("fact is %x\n", ret);
        qemu_mutex_unlock(&edu->thr_mutex);

        // 计算完成
        qatomic_and(&edu->status, ~EDU_STATUS_COMPUTING);
        smp_mb__after_rmw(); // 确保后续的read edu->status 不会排到该write之前
        // 允许计算完成后 发起中断
        if (qatomic_read(&edu->status) & EDU_STATUS_IRQFACT)
        {
            bql_lock(); // 确保可以在非主线程中调用主线程函数(pci_set_irq, msi_notify)
            printf("raise irq\n");
            edu_raise_irq(edu, FACT_IRQ);
            bql_unlock();
        }
    }
    return NULL;
}

static void pci_edu_realize(PCIDevice *pdev, Error **errp)
{
    EduState *edu = EDU(pdev);
    uint8_t *pci_conf = pdev->config;

    pci_config_set_interrupt_pin(pci_conf, 1); // 注册中断 使用INTx的引脚1

    // offset = 0: 自动分配
    // nr_vectors = 1: 支持一个MSI向量, 该设备只需要一个中断位
    // msi64bit = true: 现代系统都需要支持64位地址
    // msi_per_vector_mask: false
    // 系统不支持msi 会返回
    if (msi_init(pdev, 0, 1, true, false, errp))
        return;

    // dma timer, 回调函数是edu_dma_timer
    timer_init_ms(&edu->dma_timer, QEMU_CLOCK_VIRTUAL, edu_dma_timer, edu);

    // 创建后台线程
    qemu_mutex_init(&edu->thr_mutex);
    qemu_cond_init(&edu->thr_cond);
    qemu_thread_create(&edu->thread, "edu", edu_fact_thread, edu, QEMU_THREAD_JOINABLE);

    // 注册 1MB 的 MMIO 并 映射到 PCI BAR 0
    memory_region_init_io(&edu->mmio, OBJECT(edu), &edu_mmio_ops, edu, "edu-mmio", 1 * MiB);
    pci_register_bar(pdev, 0, PCI_BASE_ADDRESS_SPACE_MEMORY, &edu->mmio);
}

static void pci_edu_uninit(PCIDevice *pdev)
{
    EduState *edu = EDU(pdev);

    // 设置 设备停止
    qemu_mutex_lock(&edu->thr_mutex);
    edu->stopping = true;
    qemu_mutex_unlock(&edu->thr_mutex);
    qemu_cond_signal(&edu->thr_cond);
    qemu_thread_join(&edu->thread);

    qemu_cond_destroy(&edu->thr_cond);
    qemu_mutex_destroy(&edu->thr_mutex);

    timer_del(&edu->dma_timer);
    msi_uninit(pdev);
}

static void edu_instance_init(Object *obj)
{
    EduState *edu = EDU(obj);

    edu->dma_mask = (1UL << 28) - 1;
    object_property_add_uint64_ptr(obj, "dma_mask",
                                   &edu->dma_mask, OBJ_PROP_FLAG_READWRITE);
}

static void edu_class_init(ObjectClass *class, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(class);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(class);

    k->realize = pci_edu_realize;
    k->exit = pci_edu_uninit;
    k->vendor_id = PCI_VENDOR_ID_QEMU; // 厂商ID, 0x1234是QEMU官方保留的测试厂商ID
    k->device_id = 0x11e8;             // 设备ID
    /* Linux驱动通过(vendor_id, device_id)唯一标识设备
       static const struct pci_device_id edu_pci_ids[] = {
           { PCI_DEVICE(PCI_VENDOR_ID_QEMU, 0x11e8) },
           { }
       };
       MODULE_DEVICE_TABLE(pci, edu_pci_ids);
    */
    k->revision = 0x10;                            // 设备的修订版本号
    k->class_id = PCI_CLASS_OTHERS;                // 设备类别 PCI_CLASS_OTHERS属于未分类类别
    set_bit(DEVICE_CATEGORY_MISC, dc->categories); // 分组 在MISC组内显示 qemu -device help
}

static const TypeInfo edu_types[] = {
    {
        .name = TYPE_PCI_EDU_DEVICE,
        .parent = TYPE_PCI_DEVICE,
        .instance_size = sizeof(EduState),
        .instance_init = edu_instance_init,
        .class_init = edu_class_init,
        .interfaces = (const InterfaceInfo[]){
            {INTERFACE_CONVENTIONAL_PCI_DEVICE},
            {},
        },
    }};

DEFINE_TYPES(edu_types)
