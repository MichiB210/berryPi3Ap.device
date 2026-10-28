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
        move.l  %a6,-(%sp)
        move.l  %a0,-(%sp)
        move.l  %d0,-(%sp)
        jsr     _dev_init
        lea     12(%sp),%sp
        rts

        .globl  _dev_open_stub
_dev_open_stub:                         | a1 = ioreq, d0 = unit, d1 = flags, a6 = dev
        move.l  %d1,-(%sp)
        move.l  %d0,-(%sp)
        move.l  %a1,-(%sp)
        move.l  %a6,-(%sp)
        jsr     _dev_open
        lea     16(%sp),%sp
        rts

        .globl  _dev_close_stub
_dev_close_stub:                        | a1 = ioreq, a6 = dev
        move.l  %a1,-(%sp)
        move.l  %a6,-(%sp)
        jsr     _dev_close
        addq.l  #8,%sp
        rts

        .globl  _dev_expunge_stub
_dev_expunge_stub:                      | a6 = dev
        move.l  %a6,-(%sp)
        jsr     _dev_expunge
        addq.l  #4,%sp
        rts

        .globl  _dev_null_stub
_dev_null_stub:
        moveq   #0,%d0
        rts

        .globl  _dev_beginio_stub
_dev_beginio_stub:                      | a1 = ioreq, a6 = dev
        move.l  %a1,-(%sp)
        move.l  %a6,-(%sp)
        jsr     _dev_beginio
        addq.l  #8,%sp
        rts

        .globl  _dev_abortio_stub
_dev_abortio_stub:                      | a1 = ioreq, a6 = dev
        move.l  %a1,-(%sp)
        move.l  %a6,-(%sp)
        jsr     _dev_abortio
        addq.l  #8,%sp
        rts

| ---------------------------------------------------------- interrupts
| Interrupt servers: a1 = is_Data.  Z flag set (d0 = 0) = continue chain.

        .globl  _irq_server_stub
_irq_server_stub:
        move.l  %a1,-(%sp)
        jsr     _irq_server_c
        addq.l  #4,%sp
        tst.l   %d0
        rts

        .globl  _kbd_reset_stub
_kbd_reset_stub:
        move.l  %a1,-(%sp)
        jsr     _kbd_reset_c
        addq.l  #4,%sp
        tst.l   %d0
        rts

| exec ColdReboot() patch: switch the USB interrupt off, then reboot
        .globl  _coldreboot_patch
_coldreboot_patch:
        movem.l %d0-%d1/%a0-%a1,-(%sp)
        jsr     _irq_off
        movem.l (%sp)+,%d0-%d1/%a0-%a1
        move.l  _irq_old_coldreboot,-(%sp)
        rts
