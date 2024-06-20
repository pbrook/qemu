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
#include "exec/exec-all.h"

// Machine only has 2M installed, but address space is 8M?
#define MAX_DRAM_SIZE   0x800000

#define ADDR_A23        (1 << 23)

#define PROM_ADDR       0x800000
#define PROM_SIZE        0x10000

#define SRAM_ADDR       0xc00000
#define SRAM_SIZE         0x4000

#define MAPPER_PAGE_BITS    12
#define MAPPER_VBITS        (23 - MAPPER_PAGE_BITS)
#define MAPPER_RAM_ADDR     0x900000
#define MAPPER_RAM_SIZE     (8 << MAPPER_VBITS)

#define MAP_E1_R_MASK       0x8000
#define MAP_E1_W_MASK       0x4000
#define MAP_E1_X_MASK       0x2000
// Docs say we have 13 physical address bits, but we currently only implement
// 11 to avoid conflict with the directly attached bus devices.
#define MAP_E1_PHYS_MASK    0x07ff
#define MAP_E0_REF_MASK     0x0001
#define MAP_E0_DIRTY_MASK   0x0002
#define MAP_E0_USER_SHIFT   24


#define RTC_ADDR        0xd00000

struct P20MachineState {
    MachineState parent_obj;

    M68kCPU *dma_cpu;
    M68kCPU *job_cpu;
    MemoryRegion prom;
    MemoryRegion sram;
    MemoryRegion rtc;
};

#define TYPE_P20_MACHINE MACHINE_TYPE_NAME("p20")
OBJECT_DECLARE_SIMPLE_TYPE(P20MachineState, P20_MACHINE)

#define P20_SYS_UART_IRQ_MAX 4

#define P20_JOB_IPI 0
#define P20_DMA_IPI 4

#define P20_SYS_SCSI    0x0e
#define P20_SYS_MISC    0x16
#define P20_SYS_CPUC    0x18

#define MISC_NBOOT_DMA  0x8000
#define MISC_NBOOT_JOB  0x4000
#define MISC_DIS_MAP    0x0100
#define MISC_SPARE      0x0080
#define MISC_DIAG_UART  0x0040
#define MISC_HOLDMBUS   0x0020
#define MISC_NRESMB     0x0010
#define MISC_TODO       0x3e0f // not yet implemented

#define CPUC_KILL_DMA   0x01
#define CPUC_NKILL_JOB  0x02
#define CPUC_INT_DMA    0x04
#define CPUC_INT_JOB    0x08
#define CPUC_JKPD       0x40
#define CPUC_CUR_IS_JOB 0x80

/* P/20 system status and contol */
struct P20SysState {
    SysBusDevice parent_obj;

    M68kCPU *dma_cpu;
    M68kCPU *job_cpu;
    MemoryRegion iomem;
    MemoryRegion mapper;

    uint16_t scsi;
    uint16_t misc;
    uint16_t cpuc;

    p20_irq uart_irq[P20_SYS_UART_IRQ_MAX];
    uint16_t map[4 << MAPPER_VBITS];
};

#define TYPE_P20_SYS "p20-sys"
OBJECT_DECLARE_SIMPLE_TYPE(P20SysState, P20_SYS)

static int get_current_cpuid(void)
{
    if (!current_cpu) {
        return -1;
    }

    return current_cpu->cpu_index;
}

static void p20_reset_cpu(M68kCPU *cpu, bool a23)
{
    CPUState *cs = CPU(cpu);
    hwaddr vec = a23 ? ADDR_A23 : 0;
    void *p;

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

static void p20_halt_cpu(M68kCPU *cpu)
{
    CPUState *cs = CPU(cpu);

    cpu_reset(cs);
    cs->halted = true;
    cpu->env.hold_reset = true;
}

static int p20_mapper_get_pa(void *opaque, CPUM68KState *env, hwaddr *physical,
                                int *prot_p, target_ulong address,
                                int access_type)
{
    P20SysState *s = P20_SYS(opaque);
    uint16_t entry0;
    uint16_t entry1;
    int prot = 0;

    trace_p20_mapper_get_phys_addr(address, access_type);
    if (get_current_cpuid() == 1) {
        if ((s->misc & MISC_NBOOT_JOB) == 0) {
            address |= ADDR_A23;
        }
    } else {
        if ((s->misc & MISC_NBOOT_DMA) == 0) {
            address |= ADDR_A23;
        }
    }
    if ((s->misc & MISC_DIS_MAP) || (address >= ADDR_A23)) {
        if ((access_type & ACCESS_SUPER) == 0) {
            trace_p20_mapper_fail(0);
            return -1;
        }
        *physical = address;
        *prot_p = PROT_READ | PROT_WRITE | PROT_EXEC;
        return 0;
    }
    address >>= (MAPPER_PAGE_BITS - 1) & ~1;
    if (access_type & ACCESS_SUPER) {
        address |= 2 << MAPPER_VBITS;
    }
    entry0 = s->map[address];
    entry1 = s->map[address + 1];
    trace_p20_mapper_entry(entry0, entry1);
    entry0 |= MAP_E0_REF_MASK;
    if ((entry1 & MAP_E1_R_MASK) == 0) {
        prot |= PROT_READ;
    } else if ((access_type & (ACCESS_DATA | ACCESS_STORE | ACCESS_CODE)) == ACCESS_DATA) {
        // Should write/exec also require read access?
        trace_p20_mapper_fail(1);
        return -1;
    }
    if ((entry1 & MAP_E1_W_MASK) == 0) {
        if (access_type & ACCESS_STORE) {
            prot |= PROT_WRITE;
            entry0 |= MAP_E0_DIRTY_MASK;
        }
    } else if (access_type & ACCESS_STORE) {
        trace_p20_mapper_fail(2);
        return -1;
    }
    if ((entry1 & MAP_E1_X_MASK) == 0) {
        prot |= PROT_EXEC;
    } else if (access_type & ACCESS_CODE) {
        trace_p20_mapper_fail(3);
        return -1;
    }
    entry0 |= MAP_E0_DIRTY_MASK;
    if ((access_type & ACCESS_DEBUG) == 0) {
        s->map[address] = entry0;
    }
    // FIXME: User access checks
    *physical = (entry1 & MAP_E1_PHYS_MASK) << MAPPER_PAGE_BITS;
    *prot_p = prot;
    return 0;
}

static uint64_t p20_mapper_read(void *opaque, hwaddr addr, unsigned size)
{
    P20SysState *s = P20_SYS(opaque);
    uint32_t val;

    if (size == 2) {
        val = s->map[addr >> 1];
    } else if (size == 4) {
        val = p20_mapper_read(opaque, addr, 2);
        val = (val << 16) | p20_mapper_read(opaque, addr + 2, 2);
        return val;
    } else {
        val = s->map[addr >> 1];
        if (addr & 1) {
            val &= 0xff;
        } else {
            val >>= 8;
        }
    }
    trace_p20_mapper_read(addr, val);
    return val;
}

static void p20_mapper_write(void *opaque, hwaddr addr, uint64_t val,
                            unsigned size)
{
    P20SysState *s = P20_SYS(opaque);
    int idxmap;
    uint16_t e0;
    uint16_t e1;

    if (size == 4) {
        trace_p20_mapper_write32(addr, val);
        addr >>= 1;
        e0 = s->map[addr];
        e1 = s->map[addr + 1];
        if (val == ((e0 << 16) | e1)) {
            return;
        }
        s->map[addr] = val >> 16;
        s->map[addr + 1] = val;
    } else {
        if (size == 1) {
            abort();// FIXME
        }
        trace_p20_mapper_write(addr, val);
        addr >>= 1;
        if (s->map[addr] == val) {
            return;
        }
        s->map[addr] = val;
    }
    if (s->misc & MISC_DIS_MAP) {
        return;
    }
    addr >>= 1;
    if (addr & (1 << MAPPER_VBITS)) {
        idxmap = 1 << MMU_KERNEL_IDX;
        addr &= (1<< MAPPER_VBITS) - 1;
    } else {
        idxmap = 1 << MMU_USER_IDX;
    }
    tlb_flush_page_by_mmuidx_all_cpus_synced(current_cpu,
                                             addr << MAPPER_PAGE_BITS,
                                             idxmap);
}

static void p20_sys_irq_update_job(P20SysState *s)
{
    int level = 0;

    if (s->cpuc & CPUC_INT_JOB) {
        level = 4;
    }
    trace_p20_sys_irq_job(level);
    m68k_set_irq_level(s->job_cpu, level, 0);
}

static uint8_t p20_sys_irq_ack_job(void *opaque)
{
    P20SysState *s = P20_SYS(opaque);
    int vector;

    if (s->cpuc & CPUC_INT_JOB) {
        vector = 0xc1;
        //s->cpuc &= ~CPUC_INT_JOB;
        //p20_sys_irq_update_job(s);
    } else {
        vector = 0x0f;
    }
    trace_p20_sys_irq_job_ack(vector);
    return vector;
}

static void p20_sys_irq_update_dma(P20SysState *s)
{
    int level = 0;
    int n;

    for (n = 0; n < P20_SYS_UART_IRQ_MAX; n++) {
        if (s->uart_irq[n].level) {
            level = 5;
            break;
        }
    }
    if (s->cpuc & CPUC_INT_DMA) {
        level = 2;
    }
    trace_p20_sys_irq_dma(level);
    m68k_set_irq_level(s->dma_cpu, level, 0);
}

static uint8_t p20_sys_irq_ack_dma(void *opaque)
{
    P20SysState *s = P20_SYS(opaque);
    p20_irq *irq;
    int n;
    int vector = -1;

    for (n = 0; n < P20_SYS_UART_IRQ_MAX; n++) {
        irq = &s->uart_irq[n];
        if (irq->level) {
            vector = irq->ack(irq->ack_arg);
            break;
        }
    }
    if (vector < 0 && s->cpuc & CPUC_INT_DMA) {
        vector = 0xc2;
    }
    if (vector < 0) {
        vector = 0x0f;
    }
    trace_p20_sys_irq_dma_ack(vector);
    return vector;
}

static void p20_sys_uart_irq_handler(void *opaque, int irq, int level)
{
    P20SysState *s = P20_SYS(opaque);

    if (s->uart_irq[irq].level == level) {
        return;
    }
    s->uart_irq[irq].level = level;
    p20_sys_irq_update_dma(s);
}

static uint64_t p20_sys_mmio_read(void *opaque, hwaddr addr, unsigned size)
{
    P20SysState *s = P20_SYS(opaque);
    uint16_t val;

    switch (addr & ~1) {
    case P20_SYS_SCSI:
        val = s->scsi;
        break;
    case P20_SYS_MISC:
        val = s->misc;
        break;
    case P20_SYS_CPUC:
        val = s->cpuc;
        if (get_current_cpuid() == 1) {
            val |= CPUC_CUR_IS_JOB;
        }
        break;
    default:
        val = 0;
        qemu_log_mask(LOG_UNIMP, "p20_sys_mmio_read unimplemented @ 0x%"HWADDR_PRIx"\n", addr);
        break;
    }

    if (size == 1) {
        if (addr & 1) {
            val &= 0xff;
        } else {
            val >>= 8;
        }
    }
    trace_p20_sys_mmio_read(addr, val);

    return val;
}

static void p20_sys_mmio_write(void *opaque, hwaddr addr, uint64_t val,
                            unsigned size)
{
    P20SysState *s = P20_SYS(opaque);
    uint16_t mask;

    trace_p20_sys_mmio_write(addr, val);

    if (size == 1) {
        // Turn a byte write into a r-m-w
        if (addr != 0x11 && addr != 0x10) {
            uint16_t orig;
            orig = p20_sys_mmio_read(opaque, addr & ~1, size);
            if (addr & 1) {
                val |= orig & 0xff00;
            } else {
                val = (val << 8) | (orig & 0xff);
            }
        }
    }
    switch (addr & ~1) {
    case 0x00: /* NOP */
        break;
    case 0x10: /* Set LED */
        trace_p20_sys_led((int)val);
        break;
    case P20_SYS_MISC:
        val &= ~MISC_SPARE;
        mask = s->misc ^ val;
        if (mask & MISC_TODO) {
            qemu_log_mask(LOG_UNIMP,
                          "p20_sys_mmio_write MISC unimplemented bits 0x%04x\n",
                          (uint16_t)(mask & MISC_TODO));
        }
        s->misc = val;
        if (mask & (MISC_NBOOT_DMA | MISC_NBOOT_JOB | MISC_DIS_MAP)) {
            tlb_flush_all_cpus_synced(current_cpu);
        }
        break;
    case P20_SYS_CPUC:
        mask = CPUC_KILL_DMA | CPUC_NKILL_JOB | CPUC_JKPD;
        mask &= (s->cpuc ^ val);
        s->cpuc ^= mask;
        if (mask & CPUC_KILL_DMA) {
            if (val & CPUC_KILL_DMA) {
                p20_halt_cpu(s->dma_cpu);
            } else {
                p20_reset_cpu(s->dma_cpu, (s->misc & MISC_NBOOT_DMA) == 0);
            }
        }
        if (mask & CPUC_NKILL_JOB) {
            if (val & CPUC_NKILL_JOB) {
                p20_reset_cpu(s->job_cpu, (s->misc & MISC_NBOOT_JOB) == 0);
            } else {
                p20_halt_cpu(s->job_cpu);
            }
        }
        if (mask & CPUC_JKPD) {
            qemu_log_mask(LOG_UNIMP, "p20_sys_mmio_write unimplemented CPUC_JKPF\n");
        }
        break;
    case 0x20:
    case 0x40:
    case 0x60:
        s->cpuc &= ~CPUC_INT_JOB;
        p20_sys_irq_update_job(s);
        break;
    case 0x80:
        s->cpuc |= CPUC_INT_JOB;
        p20_sys_irq_update_job(s);
        break;
    case 0xa0:
        s->cpuc &= ~CPUC_INT_DMA;
        p20_sys_irq_update_dma(s);
        break;
    case 0xc0:
        s->cpuc |= CPUC_INT_DMA;
        p20_sys_irq_update_dma(s);
        break;
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
        qemu_log_mask(LOG_UNIMP, "p20_sys_mmio_write unimplemented @ 0x%"HWADDR_PRIx" val 0x%04x\n", addr, (uint16_t)val);
        break;
    }
}

static const MemoryRegionOps p20_mapper_ops = {
    .read = p20_mapper_read,
    .write = p20_mapper_write,
    .impl.max_access_size = 4,
    .endianness = DEVICE_BIG_ENDIAN,
};

static const MemoryRegionOps p20_sys_mmio_ops = {
    .read = p20_sys_mmio_read,
    .write = p20_sys_mmio_write,
    .impl.max_access_size = 2,
    .endianness = DEVICE_BIG_ENDIAN,
};

static void p20_sys_reset(DeviceState *dev)
{
    P20SysState *s = P20_SYS(dev);

    p20_reset_cpu(s->dma_cpu, true);
    p20_halt_cpu(s->job_cpu);
}

static void p20_sys_realize(DeviceState *dev, Error **errp)
{
    P20SysState *s = P20_SYS(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);
    int i;

    memory_region_init_io(&s->mapper, OBJECT(s), &p20_mapper_ops, s,
                          "p20.mapper", MAPPER_RAM_SIZE);
    sysbus_init_mmio(sbd, &s->mapper);
    memory_region_init_io(&s->iomem, OBJECT(s), &p20_sys_mmio_ops, s,
                          "p20.sys", 0x200);
    sysbus_init_mmio(sbd, &s->iomem);
    qdev_init_gpio_in_named(dev, p20_sys_uart_irq_handler, "uart-irq", 8);
    for (i = 0; i < P20_SYS_UART_IRQ_MAX; i++) {
        s->uart_irq[i].irq = qdev_get_gpio_in_named(dev, "uart-irq", i);
    }
    s->dma_cpu->env.irq_ack = p20_sys_irq_ack_dma;
    s->dma_cpu->env.irq_ack_arg = s;
    s->dma_cpu->env.emmu_get_pa = p20_mapper_get_pa;
    s->dma_cpu->env.emmu_arg = s;
    s->job_cpu->env.irq_ack = p20_sys_irq_ack_job;
    s->job_cpu->env.irq_ack_arg = s;
    s->job_cpu->env.emmu_get_pa = p20_mapper_get_pa;
    s->job_cpu->env.emmu_arg = s;
}

static Property p20_sys_properties[] = {
    DEFINE_PROP_LINK("dma-cpu", P20SysState, dma_cpu,
                     TYPE_M68K_CPU, M68kCPU *),
    DEFINE_PROP_LINK("job-cpu", P20SysState, job_cpu,
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
    m->job_cpu = M68K_CPU(cpu_create(machine->cpu_type));

    // FIXME: Validate ram_size
    /* Main DRAM at address zero */
    memory_region_add_subregion(address_space_mem, 0, machine->ram);

    memory_region_init_ram(&m->sram, NULL, "sram", SRAM_SIZE, &error_fatal);
    memory_region_add_subregion(address_space_mem, SRAM_ADDR, &m->sram);

    memory_region_init_ram(&m->rtc, NULL, "rtc", 0x80, &error_fatal);
    memory_region_add_subregion(address_space_mem, RTC_ADDR, &m->rtc);

    /* Boot rom.  */
    memory_region_init_rom(&m->prom, NULL, "prom", PROM_SIZE, &error_fatal);
    memory_region_add_subregion(address_space_mem, PROM_ADDR, &m->prom);
    rom_loaded = load_image_targphys(prom_filename, PROM_ADDR, PROM_SIZE);
    if (rom_loaded < 0) {
        error_report("Could not load rom '%s'", prom_filename);
        exit(1);
    }

    sys = P20_SYS(qdev_new(TYPE_P20_SYS));
    object_property_set_link(OBJECT(sys), "dma-cpu",
                             OBJECT(m->dma_cpu), &error_abort);
    object_property_set_link(OBJECT(sys), "job-cpu",
                             OBJECT(m->job_cpu), &error_abort);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(sys), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(sys), 0, MAPPER_RAM_ADDR);
    sysbus_mmio_map(SYS_BUS_DEVICE(sys), 1, 0xe00000);

    mk68564_create(0xa00000, serial_hd(1), serial_hd(0), &sys->uart_irq[3]);
    mk68564_create(0xa10000, NULL, NULL, &sys->uart_irq[2]);
    mk68564_create(0xa20000, NULL, NULL, &sys->uart_irq[1]);
    mk68564_create(0xa30000, NULL, NULL, &sys->uart_irq[0]);

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
