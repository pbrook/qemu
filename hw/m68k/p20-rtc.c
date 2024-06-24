/*
 * Plexus P/20 RTC (MC146818)
 *
 * Copyright (c) 2024 Paul Brook
 *
 * This code is licensed under the GPL.
 */

// FIXME: Only ram and squarewave output implemented. Actual RTC bits missing


#include "qemu/osdep.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "qemu/timer.h"
#include "hw/rtc/mc146818rtc_regs.h"
#include "hw/rtc/p20-rtc.h"
#include "trace.h"

static uint32_t rtc_periodic_clock_ticks(P20RtcState *s)
{
    int period_code;

    if (!(s->cmos_data[RTC_REG_B] & REG_B_SQWE)) {
        return 0;
     }

    period_code = s->cmos_data[RTC_REG_A] & 0x0f;

    return periodic_period_to_clock(period_code);
}

static void periodic_timer_update(P20RtcState *s, int64_t current_time)
{
    uint32_t period_clocks = rtc_periodic_clock_ticks(s);

    if (period_clocks == 0) {
        timer_del(s->periodic_timer);
    } else {
        uint64_t period_ns = periodic_clock_to_ns(period_clocks);
        s->next_periodic_time = current_time + period_ns;
        timer_mod(s->periodic_timer, s->next_periodic_time);
    }
}

static void rtc_periodic_timer(void *opaque)
{
    P20RtcState *s = opaque;

    periodic_timer_update(s, s->next_periodic_time);
    if (s->cmos_data[RTC_REG_B] & REG_B_SQWE) {
        trace_p20_rtc_tick();
        qemu_irq_pulse(s->irq);
    }
}

static void p20_rtc_write(void *opaque, hwaddr addr,
                              uint64_t data, unsigned size)
{
    P20RtcState *s = opaque;
    bool update_periodic_timer;

    if ((addr & 1) == 0) {
        return;
    } 
    addr >>= 1;
    trace_p20_rtc_write(addr, data);
    switch (addr) {
#if 0
        /* Meh. The next tick is good enough.  */
    case RTC_REG_A:
        update_periodic_timer = (s->cmos_data[RTC_REG_A] ^ data) & 0x0f;

        if (update_periodic_timer) {
            periodic_timer_update(s, qemu_clock_get_ns(rtc_clock), true);
        }
        break;
#endif
    case RTC_REG_B:
        update_periodic_timer = (s->cmos_data[RTC_REG_B] ^ data)
                                   & REG_B_SQWE;
        s->cmos_data[RTC_REG_B] = data;

        if (update_periodic_timer) {
            periodic_timer_update(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
        }
        break;
    case RTC_REG_C:
    case RTC_REG_D:
        /* cannot write to them */
        break;
    default:
        s->cmos_data[addr] = data;
        break;
    }
}

static uint64_t p20_rtc_read(void *opaque, hwaddr addr, unsigned size)
{
    P20RtcState *s = opaque;
    int ret;
    if ((addr & 1) == 0) {
        return 0;
    }
    addr >>= 1;
    switch (addr) {
#if 0
    case RTC_REG_A:
        ret = s->cmos_data[addr];
        if (update_in_progress(s)) {
            ret |= REG_A_UIP;
        }
        break;
#endif
    default:
        ret = s->cmos_data[addr];
        break;
    }
    trace_p20_rtc_read(addr, ret);
    return ret;
}

static const MemoryRegionOps p20_rtc_mmio_ops = {
    .read = p20_rtc_read,
    .write = p20_rtc_write,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 1,
    },
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void rtc_realizefn(DeviceState *dev, Error **errp)
{
    P20RtcState *s = P20_RTC(dev);
    SysBusDevice *sb = SYS_BUS_DEVICE(dev);

    s->cmos_data[RTC_REG_A] = 0x2a;
    s->cmos_data[RTC_REG_B] = 0x06;
    s->cmos_data[RTC_REG_C] = 0x00;
    s->cmos_data[RTC_REG_D] = 0;//0x80;

    s->periodic_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, rtc_periodic_timer, s);

    memory_region_init_io(&s->io, OBJECT(s), &p20_rtc_mmio_ops, s, "p20-rtc", 0x80);
    sysbus_init_mmio(sb, &s->io);
    sysbus_init_irq(sb, &s->irq);
}

static void rtc_reset(DeviceState *dev)
{
    P20RtcState *s = P20_RTC(dev);

    s->cmos_data[RTC_REG_B] &= ~REG_B_SQWE;
}

static void rtc_class_initfn(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = rtc_realizefn;
    dc->reset = rtc_reset;
    //dc->vmsd = &vmstate_rtc;
    //rc->phases.enter = rtc_reset_enter;
    //rc->phases.hold = rtc_reset_hold;
}

static const TypeInfo p20_rtc_info = {
    .name          = TYPE_P20_RTC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(P20RtcState),
    .class_init    = rtc_class_initfn,
};

static void p20_rtc_register_types(void)
{
    type_register_static(&p20_rtc_info);
}

type_init(p20_rtc_register_types)
