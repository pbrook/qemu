.. _ColdFire-m68k-System-emulator:

ColdFire and m68k System emulator
---------------------------------

Use the executable ``qemu-system-m68k`` to simulate a ColdFire or Motorola 68k based machines.

Coldfire
========

The Coldfire machines are able to boot a uClinux kernel.

The M5208EVB emulation includes the following devices:

-  MCF5208 ColdFire V2 Microprocessor (ISA A+ with EMAC).

-  Three Two on-chip UARTs.

-  Fast Ethernet Controller (FEC)

The AN5206 emulation includes the following devices:

-  MCF5206 ColdFire V2 Microprocessor.

-  Two on-chip UARTs.

Plexus P/20
===========

The Plexus P/20 emulation includes the following:

- Two m68010 CPUs (JOB and DMA)

- 16k SRAM

- Up to 8MB main system memory (default 2MB)

- 8 serial ports

- SCSI controller

- Memory mapping/protection unit (aka Mapper)

- mc146818a RTC and peristent RAM (date/time not currently implemented) 

- Requires ROM images (U17-MERGED.BIN and U15-MERGED.BIN)

Example commandline:
  qemu-system-m68k -M p20 -L /path/to/ROMs \
      -drive file=plexus-sanitized.img,format=raw,if=scsi \
      -serial stdio -display none \
      -icount 7,sleep=off,align=off \
      -global p20-rtc.file=rtc.data \
      -device scsi-tape,scsi-id=7,drive=p20tape -blockdev file,node-name=p20tape,filename=tapefile.img
