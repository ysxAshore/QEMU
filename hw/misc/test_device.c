#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "hw/core/cpu.h"
#include "exec/target_page.h"
#include "system/address-spaces.h"

// TypeInfo .name
#define TYPE_SYSBUS_TEST_DEV "test"
typedef struct TestState TestState_t;

// 生成一个名为 TEST() 的类型安全转换函数，用于将 Object * 安全地转换为 TestRomState_t *，并在类型不匹配时立即报错，防止内存错误
DECLARE_INSTANCE_CHECKER(TestState_t, TEST, TYPE_SYSBUS_TEST_DEV)

// reg define
#define REG_CHIP_ID 0x0
#define REG_RESET 0x4
#define REG_ACCESS_VA 0x8
#define REG_BUFFER_START 0x10
#define REG_BUFFER_END 0x100

#define CHIP_ID 0x20020420

// instance type
struct TestState
{
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    uint64_t chip_id;
    unsigned char buffer[REG_BUFFER_END - REG_ACCESS_VA];
};

static void reset_buffer(TestState_t *s)
{
    *(uint64_t *)((uintptr_t)s->buffer + REG_ACCESS_VA) = 0;
    for (int i = REG_BUFFER_START; i < REG_BUFFER_END; ++i)
        s->buffer[i - REG_ACCESS_VA] = 0x5a;
}

static hwaddr vaddr2hwaddr(uintptr_t vaddr)
{
    CPUState *cpu = qemu_get_cpu(0);
    hwaddr ha = cpu_get_phys_page_debug(cpu, vaddr & TARGET_PAGE_MASK) | (vaddr & ~TARGET_PAGE_MASK);
    return ha;
}

static uint64_t test_read(void *opaque, hwaddr addr, unsigned int size)
{
    TestState_t *s = opaque;
    uint64_t ret = 0;

    if (size != 1 && size != 2 && size != 4 && size != 8)
    {
        printf("unsupported read size %x (addr %lx)\n", size, addr);
        return 0;
    }

    if (addr % size != 0)
    {
        printf("warning: unaligned read addr %lx (size %x)\n", addr, size);
        return 0;
    }

    if (addr + size <= REG_RESET)
    {
        // read chip_id
        for (int i = 0; i < size; ++i)
        {
            hwaddr chip_addr = addr + i;
            uint8_t byte = (CHIP_ID >> ((REG_RESET - 1 - chip_addr) * 8)) & 0xFF;
            ret = (ret << 8) | byte;
        }
    }
    else if (addr >= REG_ACCESS_VA && addr + size <= REG_BUFFER_START)
    {
        // read guest vaddr
        uint64_t va = *(uint64_t *)((uintptr_t)s->buffer);
        printf("the vaddr is %lx, ", va);
        hwaddr ha = vaddr2hwaddr(va);
        printf("the hwaddr is %lx\n", ha);
        uint64_t value = 0;
        if (ha != (uint64_t)-1)
        {
            MemTxResult res = address_space_read(&address_space_memory, ha, MEMTXATTRS_UNSPECIFIED, &value, 8);
            if (res == MEMTX_OK)
                printf("the data is %lx\n", value);
            else
                printf("address space read failed\n");
        }
        else
            printf("vaddr to hwaddr is failed\n");
        for (int i = 0; i < size; ++i)
        {
            int shift = ((addr - REG_ACCESS_VA + i) * 8);
            uint8_t byte = (value >> shift) & 0xFF;
            ret = ((uint64_t)byte << shift) | ret;
        }
    }
    else if (addr >= REG_BUFFER_START && addr + size <= REG_BUFFER_END)
    {
        // read buffer
        for (int i = size - 1; i >= 0; --i)
        {
            hwaddr buf_idx = addr - REG_ACCESS_VA + i;
            ret = (ret << 8) | s->buffer[buf_idx];
        }
    }
    else
    {
        printf("unsupported read addr %lx size %x (out of range)\n", addr, size);
        return 0;
    }

    return ret;
}

static void test_write(void *opaque, hwaddr addr, uint64_t val, unsigned width)
{
    TestState_t *s = opaque;

    if (width != 1 && width != 2 && width != 4 && width != 8)
    {
        printf("unsupported read size %x (addr %lx)\n", width, addr);
        return;
    }

    if (addr + width > REG_BUFFER_END)
    {
        printf("write out of range: addr %lx + width %x > REG_BUFFER_END %lx\n",
               addr, width, (hwaddr)REG_BUFFER_END);
        return;
    }

    if (addr % width != 0)
    {
        printf("warning: unaligned write addr %lx (width %x)\n", addr, width);
        return;
    }

    printf("write %lx value %lx width %x\n", addr, val, width);
    if (addr == REG_RESET)
        reset_buffer(s);
    else if (addr >= REG_ACCESS_VA && addr + width <= REG_BUFFER_END)
        for (int i = 0; i < width; ++i)
        {
            hwaddr buf_idx = addr - REG_ACCESS_VA + i;
            uint8_t byte = (val >> (i * 8)) & 0xff;
            s->buffer[buf_idx] = byte;
        }
    else
        printf("unsupported write %lx value %lx width %x\n", addr, val, width);
}

static const MemoryRegionOps test_ops = {
    .read = test_read,
    .write = test_write,
    .valid.min_access_size = 1,
    .valid.max_access_size = 8,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void test_instance_init(Object *obj)
{
    TestState_t *s = TEST(obj);

    // alloc memory map region
    // 注册s->iomem 并设置回调
    memory_region_init_io(&s->iomem, obj, &test_ops, s, TYPE_SYSBUS_TEST_DEV, REG_BUFFER_END);
    // 挂载到sysbus
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    reset_buffer(s);
    s->chip_id = CHIP_ID;
}

static const TypeInfo test_info = {
    .name = TYPE_SYSBUS_TEST_DEV,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(TestState_t),
    .instance_init = test_instance_init,
};

static void test_rom_register_types(void)
{
    type_register_static(&test_info);
}

type_init(test_rom_register_types)