| berrypi3ap.device - entry and register-argument stubs
| Copyright (C) 2026 MichiB210
| SPDX-License-Identifier: GPL-2.0-or-later
|
| Exec calls device vectors with register arguments; the C code uses the
| normal stack ABI.  d0-d1/a0-a1 are scratch, everything else is preserved
| by the C functions.

        .text
        .globl  _start
_start:
        moveq   #-1,%d0                 | not an executable
        rts

        .globl  _dev_init_stub
_dev_init_stub:                         | d0 = device, a0 = seglist, a6 = SysBase
#ifdef ROMBUILD
        move.l  %a4,-(%sp)
        move.l  %d0,%a4
#endif
        move.l  %a6,-(%sp)
        move.l  %a0,-(%sp)
        move.l  %d0,-(%sp)
        jsr     _dev_init
        lea     12(%sp),%sp
#ifdef ROMBUILD
        move.l  (%sp)+,%a4
#endif
        rts

        .globl  _dev_open_stub
_dev_open_stub:                         | a1 = ioreq, d0 = unit, d1 = flags, a6 = dev
#ifdef ROMBUILD
        move.l  %a4,-(%sp)
        move.l  %a6,%a4
#endif
        move.l  %d1,-(%sp)
        move.l  %d0,-(%sp)
        move.l  %a1,-(%sp)
        move.l  %a6,-(%sp)
        jsr     _dev_open
        lea     16(%sp),%sp
#ifdef ROMBUILD
        move.l  (%sp)+,%a4
#endif
        rts

        .globl  _dev_close_stub
_dev_close_stub:                        | a1 = ioreq, a6 = dev
#ifdef ROMBUILD
        move.l  %a4,-(%sp)
        move.l  %a6,%a4
#endif
        move.l  %a1,-(%sp)
        move.l  %a6,-(%sp)
        jsr     _dev_close
        addq.l  #8,%sp
#ifdef ROMBUILD
        move.l  (%sp)+,%a4
#endif
        rts

        .globl  _dev_expunge_stub
_dev_expunge_stub:                      | a6 = dev
#ifdef ROMBUILD
        move.l  %a4,-(%sp)
        move.l  %a6,%a4
#endif
        move.l  %a6,-(%sp)
        jsr     _dev_expunge
        addq.l  #4,%sp
#ifdef ROMBUILD
        move.l  (%sp)+,%a4
#endif
        rts

        .globl  _dev_null_stub
_dev_null_stub:
        moveq   #0,%d0
        rts

        .globl  _dev_beginio_stub
_dev_beginio_stub:                      | a1 = ioreq, a6 = dev
#ifdef ROMBUILD
        move.l  %a4,-(%sp)
        move.l  %a6,%a4
#endif
        move.l  %a1,-(%sp)
        move.l  %a6,-(%sp)
        jsr     _dev_beginio
        addq.l  #8,%sp
#ifdef ROMBUILD
        move.l  (%sp)+,%a4
#endif
        rts

        .globl  _dev_abortio_stub
_dev_abortio_stub:                      | a1 = ioreq, a6 = dev
#ifdef ROMBUILD
        move.l  %a4,-(%sp)
        move.l  %a6,%a4
#endif
        move.l  %a1,-(%sp)
        move.l  %a6,-(%sp)
        jsr     _dev_abortio
        addq.l  #8,%sp
#ifdef ROMBUILD
        move.l  (%sp)+,%a4
#endif
        rts

| ---------------------------------------------------------- interrupts
| Interrupt servers: a1 = is_Data.  Z flag set (d0 = 0) = continue chain.

        .globl  _irq_server_stub
_irq_server_stub:
#ifdef ROMBUILD
        move.l  %a4,-(%sp)
        move.l  %a1,%a4
#endif
        move.l  %a1,-(%sp)
        jsr     _irq_server_c
        addq.l  #4,%sp
#ifdef ROMBUILD
        move.l  (%sp)+,%a4
#endif
        tst.l   %d0
        rts

        .globl  _kbd_reset_stub
_kbd_reset_stub:
#ifdef ROMBUILD
        move.l  %a4,-(%sp)
        move.l  %a1,%a4
#endif
        move.l  %a1,-(%sp)
        jsr     _kbd_reset_c
        addq.l  #4,%sp
#ifdef ROMBUILD
        move.l  (%sp)+,%a4
#endif
        tst.l   %d0
        rts

| exec ColdReboot() patch: switch the USB interrupt off, then reboot
        .globl  _coldreboot_patch
_coldreboot_patch:
#ifdef ROMBUILD
| ROM build: no global to read the device base from, so look it up in
| exec's device list (the driver never leaves it, dev_expunge refuses).
| a6 = SysBase (library call).  Interrupts stay off: we reboot anyway.
        subq.l  #4,%sp                  | becomes the old ColdReboot address
        movem.l %d0-%d1/%a0-%a1/%a4,-(%sp)
        jsr     -120(%a6)               | Disable
        lea     350(%a6),%a0            | SysBase->DeviceList
        lea     rom_devname(%pc),%a1
        jsr     -276(%a6)               | FindName
        move.l  %d0,%a4
        jsr     _coldreboot_rom         | irq_off(), d0 = old ColdReboot
        move.l  %d0,20(%sp)
        movem.l (%sp)+,%d0-%d1/%a0-%a1/%a4
        rts

rom_devname:
        .asciz  "berrypi3ap.device"     | = DEVNAME in dev.h
        .even

| unit task entry: a4 = FindTask(NULL)->tc_UserData (device.c sets it)
        .globl  _unit_task_stub
_unit_task_stub:
        move.l  4.w,%a6
        sub.l   %a1,%a1
        jsr     -294(%a6)               | FindTask
        move.l  %d0,%a0
        move.l  88(%a0),%a4             | tc_UserData
        jmp     _unit_task_rom
#else
        movem.l %d0-%d1/%a0-%a1,-(%sp)
        jsr     _irq_off
        movem.l (%sp)+,%d0-%d1/%a0-%a1
        move.l  _irq_old_coldreboot,-(%sp)
        rts
#endif
