/*
 * Mostek MK68564 SIO (dual UART)
 *
 * Copyright (c) 2024 Paul Brook
 *
 * This code is licensed under the GPL.
 */

/*
 * FIXME: QEMU interface:
 *  + sysbus MMIO region 0: device registers
 *  + sysbus IRQ 0: UARTINTR (combined interrupt line)
 *  + sysbus IRQ 1: UARTRXINTR (receive FIFO interrupt line)
 *  + sysbus IRQ 2: UARTTXINTR (transmit FIFO interrupt line)
 *  + sysbus IRQ 3: UARTRTINTR (receive timeout interrupt line)
 *  + sysbus IRQ 4: UARTMSINTR (momem status interrupt line)
 *  + sysbus IRQ 5: UARTEINTR (error interrupt line)
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/m68k/mk68564.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "hw/qdev-clock.h"
#include "hw/qdev-properties.h"
#include "hw/qdev-properties-system.h"
#include "migration/vmstate.h"
#include "chardev/char-fe.h"
#include "chardev/char-serial.h"
#include "qemu/log.h"
#include "qemu/module.h"

DeviceState *mk68564_create(hwaddr addr, Chardev *chra, Chardev *chrb)
{
    DeviceState *dev;
    SysBusDevice *s;

    dev = qdev_new(TYPE_MK68564);
    s = SYS_BUS_DEVICE(dev);
    qdev_prop_set_chr(dev, "chardeva", chra);
    qdev_prop_set_chr(dev, "chardevb", chrb);
    sysbus_realize_and_unref(s, &error_fatal);
    sysbus_mmio_map(s, 0, addr);

    return dev;
}

#if 0
static void mk68564_update(MK68564State *s)
{
    // FIXME
}
#endif

static uint8_t mk68564_channel_read(MK68564ChannelState *ch, int offset)
{
    switch (offset) {
    case 0: /* CMDREG */
        return ch->cmdreg;
    case 1: /* MODECTL */
        return ch->modectl;
    case 2: /* INTCTL */
        return ch->intctl;
    case 3: /* SYNC1 */
        qemu_log_mask(LOG_UNIMP, "mk68564_read: SYNC1\n");
        return 0;
    case 4: /* SYNC2 */
        qemu_log_mask(LOG_UNIMP, "mk68564_read: SYNC2\n");
        return 0;
    case 5: /* RCVCTL */
        return ch->rcvctl;
    case 6: /* XMTCTL */
        return ch->xmtctl;
    case 7: /* STAT0 */
        return 0x04; // FIXME: Indicate Rx character available
    case 8: /* STAT1 */
        return 0x01;
    case 9: /* DATARG */
        qemu_log_mask(LOG_UNIMP, "mk68564_read: DATARG\n");
        return 0;
    case 10: /* TCREG */
        return ch->tcreg;
    case 11: /* BRGCTL */
        return ch->brgctl;
    case 12: /* VECTRG */
        qemu_log_mask(LOG_UNIMP, "mk68564_read: VECTRG\n");
        return 0;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "mk68564_read: Bad offset 0x%x\n", (int)offset);
        return 0xff;
    }
}

static uint64_t mk68564_read(void *opaque, hwaddr offset,
                           unsigned size)
{
    MK68564State *s = (MK68564State *)opaque;
    // Assume we are connected to a 16-bit big-endian bus
    if ((offset & 1) == 0) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "mk68564_read: Even address 0x%x\n", (int)offset);
        return 0;
    }
    offset >>= 1;
    MK68564ChannelState *ch;
    if (offset & 0x10) {
        ch = &s->channel_b;
    } else {
        ch = &s->channel_a;
    }
    offset &= 0xf;
    return mk68564_channel_read(ch, offset);
}

static void mk68564_channel_write(MK68564ChannelState *ch, int offset, uint8_t val)
{
    switch (offset) {
    case 0: /* CMDREG */
        if (val != 0) {
            qemu_log_mask(LOG_UNIMP, "mk68564_write: CMDREG unimplemented 0x%02x\n", val);
        }
        // TODO: Loopback mode
        ch->cmdreg = val & 1;
        break;
    case 1: /* MODECTL */
        if ((val & 0x0c) == 0) {
            qemu_log_mask(LOG_UNIMP, "mk68564_write: Sync mode unimplemented\n");
        }
        // TODO: Baud rate, parity and stop bits
        ch->modectl = val;
        break;
    case 2: /* INTCTL */
        if (val != 0) {
            qemu_log_mask(LOG_UNIMP, "mk68564_write: INTCTL unimplemented bits %02x\n", val);
        }
        ch->intctl = val;
        break;
    case 3: /* SYNC1 */
        qemu_log_mask(LOG_UNIMP, "mk68564_write: SYNC1 0x%02x \n", val);
        break;
    case 4: /* SYNC2 */
        qemu_log_mask(LOG_UNIMP, "mk68564_write: SYNC2 0x%02x \n", val);
        break;
    case 5: /* RCVCTL */
        if ((val & 0x1e) != 0) {
            qemu_log_mask(LOG_UNIMP, "mk68564_write: RCVCTL unimplemented bits %02x\n", val);
        }
        // TODO: character width
        // TODO: RX enable
        ch->rcvctl = val;
        break;
    case 6: /* XMTCTL */
        if ((val & 0x30) != 0) {
            qemu_log_mask(LOG_UNIMP, "mk68564_write: XMTCTL unimplemented bits %02x\n", val);
        }
        // TODO: TX enable
        ch->xmtctl = val;
        break;
        break;
        /* 7: STAT0 and 8:STAT1 are readonly */
    case 9: /* DATARG */
        // TODO: Check if transmitter is enabled
        qemu_log_mask(LOG_UNIMP, "mk68564_write: DATARG %d\n", val);
        qemu_chr_fe_write_all(&ch->chr, &val, 1);
        break;
    case 10: /* TCREG */
        ch->tcreg = val;
        // TODO: Baud rate
        break;
    case 11: /* BRGCTL */
        ch->brgctl = val;
        break;
    case 12: /* VECTRG */
        qemu_log_mask(LOG_UNIMP, "mk68564_write: VECTRG 0x%02x \n", val);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "mk68564_write: Bad offset 0x%x\n", (int)offset);
    }
}

static void mk68564_write(void *opaque, hwaddr offset,
                        uint64_t value, unsigned size)
{
    MK68564State *s = (MK68564State *)opaque;

    if ((offset & 1) == 0) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "mk68564_write: Even address 0x%x\n", (int)offset);
        return;
    }
    offset >>= 1;
    MK68564ChannelState *ch;
    if (offset & 0x10) {
        ch = &s->channel_b;
    } else {
        ch = &s->channel_a;
    }
    offset &= 0xf;
    mk68564_channel_write(ch, offset, value);
}

static int mk68564_can_receive(void *opaque)
{
    //MK68564ChannelState *s = (MK68564ChannelState *)opaque;

    // FIXME
    return 0;
}

static void mk68564_receive(void *opaque, const uint8_t *buf, int size)
{
#if 0
    MK68564ChannelState *s = (MK68564ChannelState *)opaque;
    /*
     * In loopback mode, the RX input signal is internally disconnected
     * from the entire receiving logics; thus, all inputs are ignored,
     * and BREAK detection on RX input signal is also not performed.
     */
    if (mk68564_loopback_enabled(s)) {
        return;
    }
#endif

    qemu_log_mask(LOG_UNIMP, "mk68564: recieve\n");
}

static void mk68564_event(void *opaque, QEMUChrEvent event)
{
    if (event == CHR_EVENT_BREAK) {
        // FIXME
    }
}

static const MemoryRegionOps mk68564_ops = {
    .read = mk68564_read,
    .write = mk68564_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl.min_access_size = 1,
    .impl.max_access_size = 1,
};

#if 0
// FIXME
static const VMStateDescription vmstate_pl011 = {
    .name = "pl011",
    .version_id = 2,
    .minimum_version_id = 2,
    .post_load = pl011_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(readbuff, PL011State),
        VMSTATE_UINT32(flags, PL011State),
        VMSTATE_UINT32(lcr, PL011State),
        VMSTATE_UINT32(rsr, PL011State),
        VMSTATE_UINT32(cr, PL011State),
        VMSTATE_UINT32(dmacr, PL011State),
        VMSTATE_UINT32(int_enabled, PL011State),
        VMSTATE_UINT32(int_level, PL011State),
        VMSTATE_UINT32_ARRAY(read_fifo, PL011State, PL011_FIFO_DEPTH),
        VMSTATE_UINT32(ilpr, PL011State),
        VMSTATE_UINT32(ibrd, PL011State),
        VMSTATE_UINT32(fbrd, PL011State),
        VMSTATE_UINT32(ifl, PL011State),
        VMSTATE_INT32(read_pos, PL011State),
        VMSTATE_INT32(read_count, PL011State),
        VMSTATE_INT32(read_trigger, PL011State),
        VMSTATE_END_OF_LIST()
    },
};
#endif

static Property mk68564_properties[] = {
    DEFINE_PROP_CHR("chardeva", MK68564State, channel_a.chr),
    DEFINE_PROP_CHR("chardevb", MK68564State, channel_b.chr),
    DEFINE_PROP_END_OF_LIST(),
};

static void mk68564_init(Object *obj)
{
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    MK68564State *s = MK68564(obj);
    static int uid;

    s->channel_a.uid = uid++;
    s->channel_b.uid = uid++;
    memory_region_init_io(&s->iomem, OBJECT(s), &mk68564_ops, s, "mk68564", 0x1000);
    sysbus_init_mmio(sbd, &s->iomem);
}

static void mk68564_realize(DeviceState *dev, Error **errp)
{
    MK68564State *s = MK68564(dev);

    qemu_chr_fe_set_handlers(&s->channel_a.chr, mk68564_can_receive, mk68564_receive,
                             mk68564_event, NULL, &s->channel_a, NULL, true);
    qemu_chr_fe_set_handlers(&s->channel_b.chr, mk68564_can_receive, mk68564_receive,
                             mk68564_event, NULL, &s->channel_b, NULL, true);
}

static void mk68564_reset_channel(MK68564ChannelState *s)
{
    s->txd = 0;
    s->fifo[0] = 0;
    s->fifo[1] = 0;
    s->fifo[2] = 0;
    s->fifo[2] = 0;
    s->modectl = 0;
    s->intctl = 0;
}
static void mk68564_reset(DeviceState *dev)
{
    MK68564State *s = MK68564(dev);

    mk68564_reset_channel(&s->channel_a);
    mk68564_reset_channel(&s->channel_b);
}

static void mk68564_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = mk68564_realize;
    dc->reset = mk68564_reset;
    //FIXME: dc->vmsd = &vmstate_mk68564;
    device_class_set_props(dc, mk68564_properties);
}

static const TypeInfo mk68564_info = {
    .name          = TYPE_MK68564,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(MK68564State),
    .instance_init = mk68564_init,
    .class_init    = mk68564_class_init,
};

static void mk68564_register_types(void)
{
    type_register_static(&mk68564_info);
}

type_init(mk68564_register_types)
