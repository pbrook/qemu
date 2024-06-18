/*
 * Plexus P/20 system emulation.
 *
 * Copyright (c) 2024 Paul Brook
 *
 * This code is licensed under the GPL
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "cpu.h"
#include "hw/boards.h"
#include "hw/loader.h"
#include "hw/sysbus.h"
#include "qemu/error-report.h"
#include "qemu/log.h"
#include "sysemu/reset.h"
#include "sysemu/sysemu.h"
#include "hw/m68k/mk68564.h"
#include "hw/misc/unimp.h"
#include "trace.h"

// Machine only has 2M installed, but address space is 8M?
// FIXME: Do we need to setup aliases?
#define MAX_DRAM_SIZE   0x800000

#define PROM_ADDR       0x800000
#define PROM_SIZE        0x10000

#define SRAM_ADDR       0xc00000
#define SRAM_SIZE         0x4000

#define RTC_ADDR        0xd00000

struct P20MachineState {
    MachineState parent_obj;

    M68kCPU *dma_cpu;
    MemoryRegion prom;
    MemoryRegion sram;
    MemoryRegion rtc;
};

#define TYPE_P20_MACHINE MACHINE_TYPE_NAME("p20")
OBJECT_DECLARE_SIMPLE_TYPE(P20MachineState, P20_MACHINE)

#define P20_SYS_NIRQ 4

/* P/20 system status and contol */
struct P20SysState {
    SysBusDevice parent_obj;

    M68kCPU *dma_cpu;
    MemoryRegion iomem;

    uint16_t reg16;
    uint16_t reg18;

    p20_irq dma_irq[P20_SYS_NIRQ];
};

#define TYPE_P20_SYS "p20-sys"
OBJECT_DECLARE_SIMPLE_TYPE(P20SysState, P20_SYS)

static void dma_cpu_reset(void *opaque)
{
    M68kCPU *cpu = opaque;
    CPUState *cs = CPU(cpu);
    void *p;
    // Load the reset vectors (pc + sp) from ROM
    // FIXME: Should honor BOOT.DMA
    hwaddr vec = PROM_ADDR;

    cpu_reset(cs);
    p = rom_ptr_for_as(cs->as, vec, 8);
    if (p) {
        cpu->env.aregs[7] = ldl_p(p);
        cpu->env.pc = ldl_p(p + 4);
    } else {
        cpu->env.aregs[7] = ldl_phys(cs->as, vec);
        cpu->env.pc = ldl_phys(cs->as, vec + 4);
    }
}

static uint64_t p20_sys_mmio_read(void *opaque, hwaddr addr, unsigned size)
{
    P20SysState *s = P20_SYS(opaque);
    uint64_t val;

    if (size != 2) {
        goto bad;
    }
    switch (addr) {
    case 0x16:
        return s->reg16;
    case 0x18:
        if (size == 2) {
            return s->reg18;
        }
        return s->reg18 >> 8;
        // FIXME: bit 7 is cpu id (0=DMA, 1=JOB)
    case 0x19:
        return s->reg18 & 0xff;
    default:
    bad:
        val = 0;
        qemu_log_mask(LOG_UNIMP, "p20_sys_mmio_read unimplemented @ 0x%"HWADDR_PRIx"\n", addr);
        break;
    }

    return val;
}

static void p20_sys_mmio_write(void *opaque, hwaddr addr, uint64_t val,
                            unsigned size)
{
    P20SysState *s = P20_SYS(opaque);

    switch (addr) {
    case 0x00: /* NOP */
        break;
    case 0x10: /* Set LED */
        //DPRINTF("Set LED 0x%04x", (int)val);
        break;
    case 0x16:
        if (size != 2) {
            goto bad;
        }
        qemu_log_mask(LOG_UNIMP, "p20_sys_mmio_write Reg16 = 0x%04x\n", (uint16_t)val);
        s->reg16 = val;
        break;
    case 0x20:
    case 0x40:
    case 0x60:
    case 0x80:
    case 0xa0:
    case 0xc0:
    case 0xe0:
    case 0x100:
    case 0x120:
    case 0x140:
    case 0x160:
    case 0x180:
    case 0x1a0:
    case 0x1c0:
    case 0x1e0:
        /* Reset actions */
        qemu_log_mask(LOG_UNIMP, "p20_sys_mmio_write unimplemented reset @ 0x%"HWADDR_PRIx"\n", addr);
        break;
    default:
    bad:
        qemu_log_mask(LOG_UNIMP, "p20_sys_mmio_write unimplemented @ 0x%"HWADDR_PRIx" val 0x%04x\n", addr, (uint16_t)val);
        break;
    }
}

static void p20_sys_dma_irq_handler(void *opaque, int irq, int level)
{
    P20SysState *s = P20_SYS(opaque);

    if (level) {
        level = 5;
    }
    if (s->dma_irq[irq].level == level) {
        return;
    }
    s->dma_irq[irq].level = level;


    level = 0;
    for (irq = 0; irq < P20_SYS_NIRQ; irq++) {
        level = s->dma_irq[irq].level;
        if (level) {
            break;
        }
    }
    trace_p20_sys_dma_irq(level);
    m68k_set_irq_level(s->dma_cpu, level, 0);
}

static uint8_t p20_sys_irq_ack(void *opaque)
{
    P20SysState *s = P20_SYS(opaque);
    int irq;
    int level;
    int vector = 0x0f;

    for (irq = 0; irq < P20_SYS_NIRQ; irq++) {
        level = s->dma_irq[irq].level;
        if (level) {
            vector = s->dma_irq[irq].ack(s->dma_irq[irq].ack_arg);
            break;
        }
    }
    trace_p20_sys_dma_irq_ack(vector);
    return vector;
}

static const MemoryRegionOps p20_sys_mmio_ops = {
    .read = p20_sys_mmio_read,
    .write = p20_sys_mmio_write,
    .valid.min_access_size = 1,
    .valid.max_access_size = 2,
    .endianness = DEVICE_BIG_ENDIAN,
};

static void p20_sys_reset(DeviceState *dev)
{
    //P20SysState *s = P20_SYS(dev);
}

static void p20_sys_realize(DeviceState *dev, Error **errp)
{
    P20SysState *s = P20_SYS(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);
    int i;

    memory_region_init_io(&s->iomem, OBJECT(s), &p20_sys_mmio_ops, s,
                          "p20.sys", 0x200);
    sysbus_init_mmio(sbd, &s->iomem);
    qdev_init_gpio_in_named(dev, p20_sys_dma_irq_handler, "dma-irq", 8);
    for (i = 0; i < P20_SYS_NIRQ; i++) {
        s->dma_irq[i].irq = qdev_get_gpio_in_named(dev, "dma-irq", i);
    }
    s->dma_cpu->env.irq_ack = p20_sys_irq_ack;
    s->dma_cpu->env.irq_ack_arg = s;
}

static Property p20_sys_properties[] = {
    DEFINE_PROP_LINK("dma-cpu", P20SysState, dma_cpu,
                     TYPE_M68K_CPU, M68kCPU *),
    DEFINE_PROP_END_OF_LIST(),
};

static void p20_sys_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->desc = "P/20 System control/status";
    dc->realize = p20_sys_realize;
    dc->reset = p20_sys_reset;
    device_class_set_props(dc, p20_sys_properties);
    // FIXME: dc->vmsd = &next_pc_vmstate;
}

static const TypeInfo p20_sys_info = {
    .name = TYPE_P20_SYS,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(P20SysState),
    .class_init = p20_sys_class_init,
};

static void p20_init(MachineState *machine)
{
    P20MachineState *m = P20_MACHINE(machine);
    //ram_addr_t ram_size = machine->ram_size;
    const char *prom_filename = machine->kernel_filename;
    int rom_loaded;
    MemoryRegion *address_space_mem = get_system_memory();
    P20SysState *sys;

    m->dma_cpu = M68K_CPU(cpu_create(machine->cpu_type));
    qemu_register_reset(dma_cpu_reset, m->dma_cpu);

    // FIXME: Validate ram_size
    /* Main DRAM at address zero */
    memory_region_add_subregion(address_space_mem, 0, machine->ram);

    memory_region_init_ram(&m->sram, NULL, "sram", SRAM_SIZE, &error_fatal);
    memory_region_add_subregion(address_space_mem, SRAM_ADDR, &m->sram);

    memory_region_init_ram(&m->rtc, NULL, "rtc", 0x80, &error_fatal);
    memory_region_add_subregion(address_space_mem, RTC_ADDR, &m->rtc);

    /* Boot rom.  */
    memory_region_init_ram(&m->prom, NULL, "prom", PROM_SIZE, &error_fatal);
    memory_region_add_subregion(address_space_mem, PROM_ADDR, &m->prom);
    rom_loaded = load_image_targphys(prom_filename, PROM_ADDR, PROM_SIZE);
    if (rom_loaded < 0) {
        error_report("Could not load rom '%s'", prom_filename);
        exit(1);
    }

    sys = P20_SYS(qdev_new(TYPE_P20_SYS));
    object_property_set_link(OBJECT(sys), "dma-cpu",
                             OBJECT(m->dma_cpu), &error_abort);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(sys), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(sys), 0, 0xe00000);

    mk68564_create(0xa00000, serial_hd(1), serial_hd(0), &sys->dma_irq[3]);
    mk68564_create(0xa10000, NULL, NULL, &sys->dma_irq[2]);
    mk68564_create(0xa20000, NULL, NULL, &sys->dma_irq[1]);
    mk68564_create(0xa30000, NULL, NULL, &sys->dma_irq[0]);

    create_unimplemented_device("NOTHING", 0, 0x1000000);
}

// U9P#5 [nINTR] -> [74S240 buffer] U9D#13 && U9D#19 [nG] => U9D#7 [active high] -> U12C#3 [S374 latch D1] (C #11, nOS #1) 
//   -> U12C#2 -> U13C#6 [PAL]

static void p20_machine_class_init(ObjectClass *oc, void *data)
{
    static const char * const valid_cpu_types[] = {
        M68K_CPU_TYPE_NAME("m68010"),
        NULL
    };
    MachineClass *mc = MACHINE_CLASS(oc);
    mc->desc = "Plexus P/20";
    mc->init = p20_init;
    mc->default_cpu_type = M68K_CPU_TYPE_NAME("m68010");
    mc->valid_cpu_types = valid_cpu_types;
    mc->block_default_type = IF_SCSI;
    mc->default_ram_id = "dram";
    mc->default_ram_size = 0x200000; // 2 Mbyte
}

//DEFINE_MACHINE("plexus", p20_machine_init)
static const TypeInfo p20_machine_typeinfo = {
    .name       = MACHINE_TYPE_NAME("p20"),
    .parent     = TYPE_MACHINE,
    //.instance_init = p20_init,
    .instance_size = sizeof(P20MachineState),
    .class_init = p20_machine_class_init,
};

static void p20_machine_register_types(void)
{
    type_register_static(&p20_machine_typeinfo);
    type_register_static(&p20_sys_info);
}

type_init(p20_machine_register_types)
