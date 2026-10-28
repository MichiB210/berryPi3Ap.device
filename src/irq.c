/*
 * berrypi3ap.device - USB host controller driver for PiStorm/Emu68 (Pi 3A+)
 * Copyright (C) 2026 MichiB210
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * USB interrupt path (stage 3, step 1 - 104.3).
 *
 * How it works under Emu68 on the Pi 3 (from the Emu68 sources,
 * src/aarch64/vectors.c and intc.c, MPL-2.0 - read, not copied):
 *   - Emu68 routes the Pi's peripheral interrupts to ARM core 0.
 *   - Any IRQ on core 0 is passed to the 68k as level 6 "EXTER"
 *     (INTREQ bit 13), provided INTENA has INTEN|EXTER set.
 *   - Clearing EXTER in INTREQ clears the pending flag; the ARM IRQ
 *     stays masked until the 68k lowers its IPL again.
 * So a 68k EXTER interrupt server sees the USB interrupt.  The DWC2
 * interrupt is level triggered: the server must clear the cause in the
 * DWC2 core, otherwise it fires again at once.
 *
 * The USB interrupt is "GPU IRQ 9" of the legacy BCM2835 interrupt
 * controller (base 0x7E00B200, mapped by Emu68 like the mailbox).
 *
 * Safety:
 *   - irq_off() is called before every reboot (keyboard reset handler
 *     for Ctrl-Amiga-Amiga, ColdReboot() patch for software reboots)
 *     and at the start of hw_init(), so a left-over interrupt can never
 *     hang the next boot.
 *   - storm guard: more than IRQ_STORM interrupts within the same
 *     (micro)frame switches the interrupt off for good (counted).
 *
 * 104.3 counted SOF interrupts (8000/s at high speed - works, but each
 * one costs real Amiga bus cycles in exec's level 6 handler, so copying
 * got a few seconds slower).
 * 104.4: SOF is off again.  The unit task is now woken by the Pi's own
 * 1 MHz system timer (compare register C3, "GPU IRQ 3", not used by
 * Emu68 or the firmware): exactly one interrupt per wake-up, at the
 * microsecond it is needed, instead of timer.device.
 */
#include <proto/exec.h>
#include <exec/interrupts.h>
#include <hardware/intbits.h>
#include <devices/keyboard.h>
#include "dev.h"
#include "regs.h"

#define ARMCTRL_BASE        0xF200B200UL
#define ARMCTRL_PENDING1    0x04
#define ARMCTRL_ENABLE1     0x10
#define ARMCTRL_DISABLE1    0x1C
#define USB_IRQ_BIT         (1UL << 9)

#define TIMER_IRQ_BIT       (1UL << 3)      /* system timer match 3 */
#define TIMER1_IRQ_BIT      (1UL << 1)      /* system timer match 1 (105.0) */

#define ST_BASE             0xF2003000UL    /* 0x7E003000 */
#define ST_CS               0x00
#define ST_CLO              0x04
#define ST_C1               0x10
#define ST_C3               0x18
#define ST_M1               0x02
#define ST_M3               0x08

#define GINT_SOF            0x00000008UL
#define GAHBCFG_GLBLINTR    0x00000001UL

#define IRQ_STORM           32

#define LVO_COLDREBOOT      (-726)

static inline ULONG bswap(ULONG v)
{
    return (v >> 24) | ((v >> 8) & 0xFF00UL) | ((v << 8) & 0xFF0000UL) | (v << 24);
}

static void armctrl_wr(ULONG off, ULONG v)
{
    *(volatile ULONG *)(ARMCTRL_BASE + off) = bswap(v);
}

static void st_wr(ULONG off, ULONG v)
{
    *(volatile ULONG *)(ST_BASE + off) = bswap(v);
}

static ULONG st_rd(ULONG off)
{
    return bswap(*(volatile ULONG *)(ST_BASE + off));
}

ULONG irq_us_now(void)
{
    return st_rd(ST_CLO);
}

static struct Interrupt irq_server;
static struct Interrupt kbd_handler;
static struct IOStdReq *kbd_req;
static BOOL server_added, kbd_added;
APTR irq_old_coldreboot;            /* used by the asm patch */
static volatile BOOL irq_on;
static UWORD storm_frame, storm_cnt;

extern void irq_server_stub(void);
extern void kbd_reset_stub(void);
extern void coldreboot_patch(void);

/* Pi interrupt controller only - safe even before the USB block has
 * power (called first thing in hw_init). */
void irq_pi_off(void)
{
    armctrl_wr(ARMCTRL_DISABLE1, USB_IRQ_BIT | TIMER_IRQ_BIT | TIMER1_IRQ_BIT);
    st_wr(ST_CS, ST_M1 | ST_M3);
}

/* Switch the USB interrupt off everywhere.  Safe from any context. */
void irq_off(void)
{
    armctrl_wr(ARMCTRL_DISABLE1, USB_IRQ_BIT | TIMER_IRQ_BIT | TIMER1_IRQ_BIT);
    st_wr(ST_CS, ST_M1 | ST_M3);
    pe_enabled = FALSE;                 /* the unit task polls again */
    drv.st.pe_on = 0;
    hw_wr(GINTMSK, 0);
    hw_wr(GAHBCFG, hw_rd(GAHBCFG) & ~GAHBCFG_GLBLINTR);
    hw_wr(GINTSTS, 0xFFFFFFFFUL);
    irq_on = FALSE;
    drv.st.irq_enabled = 0;
}

/* EXTER interrupt server.  Returns 0 so the rest of the chain (CIA-B
 * and others) always runs. */
ULONG irq_server_c(APTR data)
{
    ULONG s;
    UWORD fr;

    if (!irq_on)
        return 0;

    /* system timer: wake the unit task */
    if (st_rd(ST_CS) & ST_M3) {
        st_wr(ST_CS, ST_M3);
        drv.st.irq_count++;
        drv.st.irq_timer++;
        Signal(drv.task, drv.wake_sigmask);
    }

    /* system timer C1: periodic engine */
    if (st_rd(ST_CS) & ST_M1) {
        st_wr(ST_CS, ST_M1);
        drv.st.irq_count++;
        pe_on_timer();
    }

    s = hw_rd(GINTSTS) & hw_rd(GINTMSK);
    if (!s)
        return 0;

    drv.st.irq_count++;
    if (s & GINT_HCHINT) {
        /* only channel 1 has its interrupt unmasked */
        ULONG hc = hw_rd(HCINT(1));
        hw_wr(HCINT(1), hc);
        if (hc & HCINT_CHHLTD)
            pe_on_chan(hc);
        s &= ~GINT_HCHINT;
    }
    if (s & GINT_SOF) {
        hw_wr(GINTSTS, GINT_SOF);
        drv.st.irq_sof++;
    }
    if (s & ~GINT_SOF) {
        /* not expected in this step: mask it so it cannot repeat */
        drv.st.irq_other++;
        hw_wr(GINTMSK, hw_rd(GINTMSK) & ~(s & ~GINT_SOF));
        hw_wr(GINTSTS, s & ~GINT_SOF);
    }

    fr = hw_frnum();
    if (fr == storm_frame) {
        if (++storm_cnt > IRQ_STORM) {
            irq_off();
            drv.st.irq_storm++;
        }
    } else {
        storm_frame = fr;
        storm_cnt = 0;
    }
    return 0;
}

/* Ctrl-Amiga-Amiga: switch off, then tell keyboard.device we are done
 * (done from the unit task, see irq_reset_done). */
ULONG kbd_reset_c(APTR data)
{
    irq_off();
    Signal(drv.task, drv.reset_sigmask);
    return 0;
}

void irq_reset_done(void)
{
    if (kbd_added) {
        kbd_req->io_Command = KBD_RESETHANDLERDONE;
        kbd_req->io_Data = &kbd_handler;
        DoIO((struct IORequest *)kbd_req);
    }
}

/* called by the unit task after hw_init() */
void irq_init(void)
{
    struct MsgPort *mp;

    /* reboot protection first */
    mp = CreateMsgPort();
    if (mp) {
        kbd_req = (struct IOStdReq *)CreateIORequest(mp, sizeof(struct IOStdReq));
        if (kbd_req && !OpenDevice((STRPTR)"keyboard.device", 0, (struct IORequest *)kbd_req, 0)) {
            kbd_handler.is_Node.ln_Type = NT_INTERRUPT;
            kbd_handler.is_Node.ln_Pri = 16;
            kbd_handler.is_Node.ln_Name = (char *)DEVNAME;
            kbd_handler.is_Code = (void (*)(void))kbd_reset_stub;
            kbd_handler.is_Data = NULL;
            kbd_req->io_Command = KBD_ADDRESETHANDLER;
            kbd_req->io_Data = &kbd_handler;
            DoIO((struct IORequest *)kbd_req);
            kbd_added = TRUE;
        }
    }

    Forbid();
    irq_old_coldreboot = SetFunction((struct Library *)SysBase, LVO_COLDREBOOT,
                                     (APTR)coldreboot_patch);
    Permit();

    /* the server */
    irq_server.is_Node.ln_Type = NT_INTERRUPT;
    irq_server.is_Node.ln_Pri = 127;
    irq_server.is_Node.ln_Name = (char *)DEVNAME;
    irq_server.is_Code = (void (*)(void))irq_server_stub;
    irq_server.is_Data = NULL;
    AddIntServer(INTB_EXTER, &irq_server);
    server_added = TRUE;

    /* enable: system timer C3 (task wake-up), system timer C1 and the
     * channel 1 halt interrupt (periodic engine, 105.0) */
    hw_wr(GINTSTS, 0xFFFFFFFFUL);
    hw_wr(HCINT(1), 0xFFFFFFFFUL);
    hw_wr(HCINTMSK(1), HCINT_CHHLTD);
    hw_wr(HAINTMSK, 0x2);
    hw_wr(GINTMSK, GINT_HCHINT);
    st_wr(ST_CS, ST_M1 | ST_M3);
    irq_on = TRUE;
    drv.st.irq_enabled = 1;
    hw_wr(GAHBCFG, hw_rd(GAHBCFG) | GAHBCFG_GLBLINTR);
    pe_enabled = TRUE;
    drv.st.pe_on = 1;
    armctrl_wr(ARMCTRL_ENABLE1, TIMER_IRQ_BIT | TIMER1_IRQ_BIT | USB_IRQ_BIT);
}

/* periodic engine timer: fire 'us' from now (never in the past) */
void irq_c1_at(ULONG us)
{
    ULONG target;
    if (us < 20)
        us = 20;
    for (;;) {
        st_wr(ST_CS, ST_M1);
        target = st_rd(ST_CLO) + us;
        st_wr(ST_C1, target);
        if ((LONG)(st_rd(ST_CLO) - target) < 0)
            break;
        us = 20;                        /* passed already: try again */
    }
}

BOOL irq_usable(void)
{
    return irq_on && drv.wake_sigmask;
}

/* Arm a one-shot wake-up 'us' microseconds from now.  Returns the
 * target time.  If the target has already passed when the compare
 * register is written (it only matches on equality), signal at once. */
ULONG irq_wake_in(ULONG us)
{
    ULONG target;

    st_wr(ST_CS, ST_M3);
    target = st_rd(ST_CLO) + us;
    st_wr(ST_C3, target);
    if ((LONG)(st_rd(ST_CLO) - target) >= 0) {
        st_wr(ST_CS, ST_M3);
        drv.st.irq_wake_now++;
        Signal(drv.task, drv.wake_sigmask);
    }
    return target;
}
