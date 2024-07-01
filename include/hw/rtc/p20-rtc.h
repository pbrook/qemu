#ifndef HW_RTC_P20_RTC_H
#define HW_RTC_P20_RTC_H

#include "hw/sysbus.h"
#include "qom/object.h"
#include "hw/irq.h"
#include "qemu/timer.h"

#define TYPE_P20_RTC "p20-rtc"
OBJECT_DECLARE_SIMPLE_TYPE(P20RtcState, P20_RTC)

struct P20RtcState {
    SysBusDevice parent_obj;

    MemoryRegion io;
    uint8_t cmos_data[64];
    qemu_irq irq;
    /* periodic timer */
    QEMUTimer *periodic_timer;
    int64_t next_periodic_time;

    char *filename;
};

#endif /* HW_RTC_P20_RTC_H */
