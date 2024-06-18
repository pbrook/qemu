/*
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2 or later, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef HW_MK68564_H
#define HW_MK68564_H

#include "hw/sysbus.h"
#include "chardev/char-fe.h"
#include "qom/object.h"

#define TYPE_MK68564 "mk68564"
OBJECT_DECLARE_SIMPLE_TYPE(MK68564State, MK68564)

typedef struct {
    int level;
    qemu_irq irq;
    uint8_t (*ack)(void *);
    void *ack_arg;
} p20_irq;

typedef struct {
    MK68564State *sio;
    uint8_t cmdreg;
    uint8_t modectl;
    uint8_t intctl;
    uint8_t rcvctl;
    uint8_t xmtctl;
    uint8_t stat0;
    uint8_t rxdata;
    uint8_t tcreg;
    uint8_t brgctl;
    CharBackend chr;
    int id;
} MK68564ChannelState;

struct MK68564State {
    SysBusDevice parent_obj;

    MK68564ChannelState channel_a;
    MK68564ChannelState channel_b;
    MemoryRegion iomem;
    p20_irq *irq;

    uint8_t vectrg;
    uint8_t pending;
    uint8_t irq_mask;
};

DeviceState *mk68564_create(hwaddr addr, Chardev *chra, Chardev *chrb, p20_irq *irq);
#endif
