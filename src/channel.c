/*
 * berrypi3ap.device - USB host controller driver for PiStorm/Emu68 (Pi 3A+)
 * Copyright (C) 2026 MichiB210
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Host channel handling.  Stage 1 uses channel 0 only, polled.
 * Field programming follows the DWC2 rules found in USB57..USB102 and in
 * Linux dwc2 hcd.c (dwc2_hc_init / dwc2_hc_start_transfer):
 *   - split periodic transfers use EC/MC = 3, everything else 1
 *   - periodic transfers target the next (micro)frame via ODDFRM
 *   - in buffer-DMA mode the channel halts (CHHLTD) on completion and on
 *     every error, so CHHLTD is the event we wait for
 */
#include <proto/exec.h>
#include "dev.h"
#include "regs.h"

#define CH 0

/* Halt the channel.  Returns the HCINT bits seen while halting: a
 * transaction can still complete in the moment we stop the channel
 * (103.7), and the caller must not lose that completion. */
ULONG chan_halt(void)
{
    ULONG n, seen = 0;
    ULONG c = hw_rd(HCCHAR(CH));

    if (c & HCCHAR_CHENA) {
        hw_wr(HCCHAR(CH), (c | HCCHAR_CHDIS | HCCHAR_CHENA));
        for (n = 0; n < 0x200000UL; n++) {
            ULONG v = hw_rd(HCINT(CH));
            seen |= v;
            if (v & HCINT_CHHLTD)
                break;
            if (!(hw_rd(HCCHAR(CH)) & HCCHAR_CHENA))
                break;
        }
    }
    seen |= hw_rd(HCINT(CH));
    hw_wr(HCINT(CH), 0xFFFFFFFFUL);
    return seen;
}

/* Program and enable channel 'ch'.  The caller makes sure it is idle. */
static void chan_program(ULONG ch, const struct chan_req *c)
{
    ULONG hcchar;

    hw_wr(HCINT(ch), 0xFFFFFFFFUL);
    /* channel 1 (periodic engine, 105.0) reports its halt by interrupt */
    hw_wr(HCINTMSK(ch), ch == 1 ? HCINT_CHHLTD : 0);

    if (c->split)
        hw_wr(HCSPLT(ch), HCSPLT_SPLTENA | HCSPLT_XACTPOS_ALL |
                          HCSPLT_HUBADDR(c->hub) | HCSPLT_PRTADDR(c->port) |
                          (c->complete ? HCSPLT_COMPSPLT : 0));
    else
        hw_wr(HCSPLT(ch), 0);

    hw_wr(HCTSIZ(ch), c->hctsiz);
    hw_wr(HCDMA(ch), c->bus);

    hcchar = HCCHAR_MPS(c->mps) | HCCHAR_EPNUM(c->ep) |
             HCCHAR_EPTYPE(c->eptype) | HCCHAR_DEVADDR(c->addr);
    if (c->in)
        hcchar |= HCCHAR_EPDIR_IN;
    if (c->lowspeed)
        hcchar |= HCCHAR_LSDEV;
    if (c->split && (c->eptype == EPTYPE_INTR || c->eptype == EPTYPE_ISOC))
        hcchar |= HCCHAR_MC(3);
    else
        hcchar |= HCCHAR_MC(1);

    if (c->periodic_next) {
        /* odd/even of the frame we want to be on the wire in */
        if (((hw_frnum() + 1) & 1))
            hcchar |= HCCHAR_ODDFRM;
    }

    hw_wr(HCCHAR(ch), (hcchar | HCCHAR_CHENA) & ~HCCHAR_CHDIS);
}

void chan_start(const struct chan_req *c)
{
    ULONG n;

    /* never re-arm a channel that is still enabled */
    for (n = 0; n < 0x20000UL; n++)
        if (!(hw_rd(HCCHAR(CH)) & HCCHAR_CHENA))
            break;
    if (hw_rd(HCCHAR(CH)) & HCCHAR_CHENA)
        chan_halt();
    chan_program(CH, c);
}

/* ---- channel 1: periodic engine, driven from the interrupt (105.0) */

void chan1_start(const struct chan_req *c)
{
    chan_program(1, c);
}

/* ask channel 1 to stop; the halt is reported by interrupt */
void chan1_disable(void)
{
    ULONG c = hw_rd(HCCHAR(1));
    if (c & HCCHAR_CHENA)
        hw_wr(HCCHAR(1), c | HCCHAR_CHDIS | HCCHAR_CHENA);
}

BOOL chan1_busy(void)
{
    return (hw_rd(HCCHAR(1)) & HCCHAR_CHENA) ? TRUE : FALSE;
}

ULONG chan1_hctsiz(void)
{
    return hw_rd(HCTSIZ(1));
}

/*
 * Wait for the channel to halt.  Returns all HCINT bits seen; adds
 * HCINT_SW_TIMEOUT if it did not halt within max_ticks (micro)frames.
 * The channel is left running on timeout; the caller decides to halt.
 */
ULONG chan_wait(ULONG max_ticks)
{
    ULONG seen = 0, n, elapsed = 0;
    UWORD last = hw_frnum();

    for (n = 0; ; n++) {
        ULONG v = hw_rd(HCINT(CH));
        seen |= v;
        if (v & HCINT_CHHLTD)
            return seen;
        if ((n & 0xFF) == 0xFF) {
            UWORD now = hw_frnum();
            elapsed += hw_frame_diff(now, last);    /* wrap-safe */
            last = now;
            if (elapsed > max_ticks)
                return seen | HCINT_SW_TIMEOUT;
            if (n > 0x00800000UL)       /* frame counter stopped (USB57 limit) */
                return seen | HCINT_SW_TIMEOUT;
        }
    }
}

ULONG chan_hctsiz(void)
{
    return hw_rd(HCTSIZ(CH));
}
