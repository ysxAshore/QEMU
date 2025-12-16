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
#define REG_RESET 0x8
#define REG_BUFFER_START 0x10
#define REG_BUFFER_END 0x100

#define CHIP_ID 0x20020420

// instance type
struct TestState
{
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    uint64_t chip_id;
    unsigned char buffer[REG_BUFFER_END - REG_BUFFER_START];
};

static void reset_buffer(TestState_t *s)
{
    for (int i = REG_BUFFER_START; i < REG_BUFFER_END; ++i)
        s->buffer[i - REG_BUFFER_START] = 0x5a;
}

static uint64_t test_read(void *opaque, hwaddr addr, unsigned int size)
{
    printf("read\n");
    TestState_t *s = opaque;
    if (addr < REG_CHIP_ID + 0x4)
        return (CHIP_ID >> addr * 8) & 0xff;
    else if (addr >= REG_BUFFER_START && addr < REG_BUFFER_END)
        return s->buffer[addr - REG_BUFFER_START];
    else
    {
        printf("unsupported read addr %lx size %x\n", addr, size);
        return 0;
    }
}

static void test_write(void *opaque, hwaddr addr, uint64_t val, unsigned width)
{
    TestState_t *s = opaque;
    switch (addr)
    {
    case REG_CHIP_ID:
        break;
    case REG_RESET:
        reset_buffer(s);
        break;
    default:
        if (addr >= REG_BUFFER_START && addr < REG_BUFFER_END)
            s->buffer[addr - REG_BUFFER_START] = (unsigned char)val;
        else
            printf("unsupported read write %lx value %lx width %x\n", addr, val, width);
    }
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