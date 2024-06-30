/*
 * Mostek MK68564 SIO (dual UART)
 *
 * Copyright (c) 2024 Paul Brook
 *
 * This code is licensed under the GPL.
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
#include "trace.h"

// Status register 0 bits [sic]
#define STAT0_RX_CHAR_AVAIL     0x01
#define STAT0_INERPT_PENDING    0x02
#define STAT0_TX_BUFR_EMPTY     0x04
#define STAT0_DCD               0x08
#define STAT0_HUNT              0x10
#define STAT0_CTS               0x20
#define STAT0_UNDERRUN          0x40
#define STAT0_BREAK             0x80

#define INTVEC_TX_BUFFER_EMPTY  0
#define INTVEC_STATUS_CHANGE    1
#define INTVEC_RX_AVAIL         2
#define INTVEC_SPECIAL          3

static void mk68564_reset_channel(MK68564ChannelState *ch);
static void mk68564_receive(void *opaque, const uint8_t *buf, int size);

static uint8_t mk68564_ack_irq(void *opaque);

DeviceState *mk68564_create(hwaddr addr, Chardev *chra, Chardev *chrb, p20_irq *irq)
{
    DeviceState *dev;
    SysBusDevice *s;

    dev = qdev_new(TYPE_MK68564);
    s = SYS_BUS_DEVICE(dev);
    qdev_prop_set_chr(dev, "chardeva", chra);
    qdev_prop_set_chr(dev, "chardevb", chrb);
    sysbus_realize_and_unref(s, &error_fatal);
    sysbus_mmio_map(s, 0, addr);
    MK68564(s)->irq = irq;
    irq->ack = mk68564_ack_irq;
    irq->ack_arg = MK68564(dev);

    return dev;
}

static void mk68564_update(MK68564State *s)
{
    uint8_t active = s->pending & ~s->irq_mask;
    if (active & 0xf0) {
        s->channel_a.stat0 |= STAT0_INERPT_PENDING;
    } else {
        s->channel_a.stat0 &= ~STAT0_INERPT_PENDING;
    }
    if (active & 0x0f) {
        s->channel_b.stat0 |= STAT0_INERPT_PENDING;
    } else {
        s->channel_b.stat0 &= ~STAT0_INERPT_PENDING;
    }
    qemu_set_irq(s->irq->irq, active != 0);
}

static void mk68564_channel_raise_irq(MK68564ChannelState *ch, int vec)
{
    uint8_t mask = 1 << vec;
    if (ch->id == 0) {
        mask <<= 4;
    }
    ch->sio->pending |= mask;
    mk68564_update(ch->sio);
}

static void mk68564_channel_mask_irq(MK68564ChannelState *ch, int vec)
{
    uint8_t mask = 1 << vec;
    if (ch->id == 0) {
        mask <<= 4;
    }
    ch->sio->pending &= ~mask;
    ch->sio->irq_mask |= mask;
    mk68564_update(ch->sio);
}

static void mk68564_channel_unmask_irq(MK68564ChannelState *ch, int vec)
{
    uint8_t mask = 1 << vec;
    if (ch->id == 0) {
        mask <<= 4;
    }
    ch->sio->irq_mask &= ~mask;
    mk68564_update(ch->sio);
}

static uint8_t mk68564_ack_irq(void *opaque)
{
    MK68564State *s = (MK68564State *)opaque;
    uint8_t intctl = s->channel_a.intctl | s->channel_b.intctl;
    uint8_t active = s->pending & ~s->irq_mask;
    int vector;
    uint8_t mask;
    if (s->pending & 0xf0) {
        mask = 0x10;
        vector = 4;
    } else {
        mask = 0x01;
        vector = 0;
    }
    if (active & (mask << INTVEC_SPECIAL)) {
        mask <<= INTVEC_SPECIAL;
        vector += INTVEC_SPECIAL;
    } else if (active & (mask << INTVEC_RX_AVAIL)) {
        mask <<= INTVEC_RX_AVAIL;
        vector += INTVEC_RX_AVAIL;
    } else if (active & (mask << INTVEC_TX_BUFFER_EMPTY)) {
        mask <<= INTVEC_TX_BUFFER_EMPTY;
        vector += INTVEC_TX_BUFFER_EMPTY;
    } else {
        mask <<= INTVEC_STATUS_CHANGE;
        vector += INTVEC_STATUS_CHANGE;
    }
    s->pending &= ~mask;
    mk68564_update(s);

    if (intctl & 0x04) {
        vector |= s->vectrg & ~7;
    } else {
        vector = s->vectrg;
    }
    return vector;
}

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
        return ch->stat0;
    case 8: /* STAT1 */
        /* ROM selftest uses some mismatched loopback settings to
           test the parity and stop bits */
        if (ch->cmdreg == 1) {
            if (ch->xmtctl == 0x85 && ch->rcvctl == 0xc1) {
                return 0x11;
            }
            if (ch->xmtctl == 0xc5 && ch->rcvctl == 0x41) {
                return 0x41;
            }
        }
        return 0x01;
    case 9: /* DATARG */
        ch->stat0 &= ~STAT0_RX_CHAR_AVAIL;
        return ch->rxdata;
    case 10: /* TCREG */
        return ch->tcreg;
    case 11: /* BRGCTL */
        return ch->brgctl;
    case 12: /* VECTRG */
        return ch->sio->vectrg;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "mk68564_read: Bad offset 0x%x\n", (int)offset);
        return 0xff;
    }
}

static uint64_t mk68564_read(void *opaque, hwaddr offset,
                           unsigned size)
{
    uint8_t val;

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
    val = mk68564_channel_read(ch, offset & 0xf);
    trace_mk68564_read(offset, val);
    return val;
}

static void mk68564_channel_write(MK68564ChannelState *ch, int offset, uint8_t val)
{
    trace_mk68564_write(offset, val);
    switch (offset) {
    case 0: /* CMDREG */
        if (val & 0xc0) {
            qemu_log_mask(LOG_UNIMP, "mk68564_write: CMDREG unimplemented CRC reset %d\n", val >> 6);
        }
        int cmd = (val >> 3) & 7;
        switch (cmd) {
        case 0: /* no-op */
        case 7:
            break;
        case 3: /* reset channel */
            mk68564_reset_channel(ch);
            break;
        default:
            qemu_log_mask(LOG_UNIMP, "mk68564_write: CMDREG unimplemented command %d\n", cmd);
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
        if ((val & 0xe5) != 0) {
            qemu_log_mask(LOG_UNIMP, "mk68564_write: INTCTL unimplemented bits %02x\n", val);
        }
        ch->intctl = val;
        if ((val & 0x18) == 0) {
            mk68564_channel_mask_irq(ch, INTVEC_RX_AVAIL);
            mk68564_channel_mask_irq(ch, INTVEC_SPECIAL);
        } else {
            mk68564_channel_unmask_irq(ch, INTVEC_RX_AVAIL);
            mk68564_channel_unmask_irq(ch, INTVEC_SPECIAL);
        }
        mk68564_update(ch->sio);
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
        /* 7: STAT0 and 8:STAT1 are readonly */
    case 9: /* DATARG */
        if ((ch->xmtctl & 1) == 0) {
            qemu_log_mask(LOG_UNIMP, "mk68564_write: DATARG with TX disabled\n");
        }
        // Only for debugging purposes!
        //qemu_log_mask(LOG_UNIMP, "mk68564_write: DATARG %d %c\n", val, val);
        trace_mk68564_char_out(val);
        if (ch->cmdreg) {
            /* Loopback */
            mk68564_receive(ch, &val, 0);
        } else {
            qemu_chr_fe_write_all(&ch->chr, &val, 1);
        }
        if (ch->intctl & 1) {
            mk68564_channel_raise_irq(ch, INTVEC_TX_BUFFER_EMPTY);
        }
        break;
    case 10: /* TCREG */
        ch->tcreg = val;
        // TODO: Baud rate
        break;
    case 11: /* BRGCTL */
        ch->brgctl = val;
        break;
    case 12: /* VECTRG */
        ch->sio->vectrg = val;
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
    MK68564ChannelState *ch = (MK68564ChannelState *)opaque;

    if (ch->cmdreg) {
        /* Loopback mode. Stalling probably more useful than discarding it */
        return 0;
    }
    if ((ch->rcvctl & 1) == 0) {
        return 0;
    }
    if ((ch->stat0 & STAT0_RX_CHAR_AVAIL) != 0) {
        return 0;
    }
    // Real hardware has a 3-byte fifo, but we just implement a single entry
    return 1;
}

static void mk68564_receive(void *opaque, const uint8_t *buf, int size)
{
    MK68564ChannelState *ch = (MK68564ChannelState *)opaque;

    ch->rxdata = *buf;
    ch->stat0 |= STAT0_RX_CHAR_AVAIL;
    mk68564_channel_raise_irq(ch, INTVEC_RX_AVAIL);
    mk68564_update(ch->sio);
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

    s->channel_a.sio = s;
    s->channel_b.sio = s;
    s->channel_a.id = 0;
    s->channel_b.id = 1;
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

static void mk68564_reset_channel(MK68564ChannelState *ch)
{
    ch->cmdreg = 0;
    ch->modectl = 0;
    ch->intctl = 0;
    ch->rcvctl = 0;
    ch->xmtctl = 0;
    ch->stat0 = 0x04; // ???
    ch->rxdata = 0;
    ch->tcreg = 0;
    ch->brgctl = 0;
    mk68564_update(ch->sio);
}
static void mk68564_reset(DeviceState *dev)
{
    MK68564State *s = MK68564(dev);

    mk68564_reset_channel(&s->channel_a);
    mk68564_reset_channel(&s->channel_b);
    s->vectrg = 0x0f;
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
