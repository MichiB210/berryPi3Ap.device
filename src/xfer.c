/*
 * berrypi3ap.device - USB host controller driver for PiStorm/Emu68 (Pi 3A+)
 * Copyright (C) 2026 MichiB210
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Transfer engines (stage 1: one channel, polled, but never blocking the
 * whole driver: bulk runs in time slices, interrupt endpoints are polled
 * one transaction at a time).
 *
 * Rules taken from Linux dwc2 (hcd.c, hcd_intr.c; GPL-2.0+ OR BSD-3-Clause),
 * re-implemented here:
 *   - IN transfers reserve whole max packets; actual = reserved - residual
 *   - after a halt before completion, transferred = (start_pkts - pktcnt)*mps
 *   - next DATA PID is taken from HCTSIZ after the transfer
 *   - transaction errors are retried up to 3 times before failing
 *   - periodic split: when all complete-splits answer NYET this is NOT an
 *     error, the poll is simply repeated later (dwc2_hc_nyet_intr)
 *   - high-speed OUT after NYET uses the PING protocol (HCTSIZ.DoPng)
 * Split sequencing (SSPLIT -> one (micro)frame -> up to 3 CSPLITs, NAK on
 * CSPLIT restarts the SSPLIT) is the USB66..USB73 work, found on hardware.
 */
#include <string.h>
#include <proto/exec.h>
#include "dev.h"
#include "regs.h"

#define HCTSIZ_DOPNG        0x80000000UL
#define XACT_RETRIES        3
#define CTRL_TIMEOUT_MS     5000
#define SPLIT_SS_BUDGET     64
#define SPLIT_CS_BUDGET_NP  64      /* non-periodic CSPLIT NYET budget */
#define SPLIT_CS_TRIES_P    6       /* periodic: complete-split slots, as long
                                       as they stay in the start-split's frame */


/* ------------------------------------------------------------ toggles */

static inline UWORD pipe_idx(UBYTE addr, UBYTE ep)
{
    return (UWORD)(((addr & 0x7F) << 4) | (ep & 0xF));
}

BOOL tog_get(UBYTE addr, UBYTE ep, BOOL in)
{
    UWORD i = pipe_idx(addr, ep);
    UBYTE *t = in ? drv.tog_in : drv.tog_out;
    return (t[i >> 3] >> (i & 7)) & 1;
}

/* 105.0: the periodic engine changes toggles from the interrupt, so
 * every read-modify-write runs with interrupts disabled. */
void tog_set(UBYTE addr, UBYTE ep, BOOL in, BOOL data1)
{
    UWORD i = pipe_idx(addr, ep);
    UBYTE *t = in ? drv.tog_in : drv.tog_out;
    Disable();
    if (data1)
        t[i >> 3] |= (UBYTE)(1 << (i & 7));
    else
        t[i >> 3] &= (UBYTE)~(1 << (i & 7));
    Enable();
}

void tog_clear_dev(UBYTE addr)
{
    UWORD i = (UWORD)((addr & 0x7F) << 1);
    Disable();
    drv.tog_in[i] = drv.tog_in[i + 1] = 0;
    drv.tog_out[i] = drv.tog_out[i + 1] = 0;
    Enable();
}

static ULONG pid_of(BOOL data1)
{
    return data1 ? HCTSIZ_PID_DATA1 : HCTSIZ_PID_DATA0;
}

static BOOL pid_is_data1(ULONG hctsiz)
{
    return (hctsiz & HCTSIZ_PID_MASK) == HCTSIZ_PID_DATA1;
}

/* ------------------------------------------------------------- errors */

UWORD map_hcint_error(ULONG hcint)
{
    if (hcint & HCINT_STALL)
        return UHIOERR_STALL;
    if (hcint & HCINT_BBLERR)
        return UHIOERR_OVERFLOW;
    if (hcint & HCINT_XACTERR)
        return UHIOERR_TIMEOUT;         /* no/garbled handshake */
    if (hcint & HCINT_SW_TIMEOUT)
        return UHIOERR_TIMEOUT;
    return UHIOERR_HOSTERROR;           /* AHB, toggle, overrun, other */
}

/* NAK timeout bookkeeping in milliseconds (REQ_START = ms + 1) */
static void nak_progress(struct IOUsbHWReq *io)
{
    REQ_START(io) = hw_ms_now() + 1;
}

static BOOL nak_expired(struct IOUsbHWReq *io)
{
    if (!(io->iouh_Flags & UHFF_NAKTIMEOUT) || !io->iouh_NakTimeout)
        return FALSE;
    if (REQ_START(io) == 0) {
        nak_progress(io);
        return FALSE;
    }
    return (hw_ms_now() + 1 - REQ_START(io)) >= io->iouh_NakTimeout;
}

static void set_err(struct IOUsbHWReq *io, UWORD err, ULONG hcint)
{
    io->iouh_Req.io_Error = (BYTE)err;
    io->iouh_ExtError = (UWORD)hcint;
}

/* ------------------------------------------------------ routing helpers */

/* Split only if requested, target not high speed, and the device on the
 * Pi's port is a high-speed hub (USB65 gate). */
static BOOL want_split(struct IOUsbHWReq *io)
{
    if (!(io->iouh_Flags & UHFF_SPLITTRANS))
        return FALSE;
    if (io->iouh_Flags & UHFF_HIGHSPEED)
        return FALSE;
    if (drv.port_speed != 0)
        return FALSE;
    if (io->iouh_SplitHubAddr == 0 || io->iouh_SplitHubAddr > 127)
        return FALSE;
    if (io->iouh_SplitHubPort == 0 || io->iouh_SplitHubPort > 127)
        return FALSE;
    return TRUE;
}

/* A low-speed device behind a full-speed hub needs PRE packets, which this
 * PHY cannot generate (Linux refuses this too). */
static BOOL ls_behind_fs_hub(struct IOUsbHWReq *io, BOOL split)
{
    return !split && (io->iouh_Flags & UHFF_LOWSPEED) && drv.port_speed == 1;
}

static void base_req(struct chan_req *c, struct IOUsbHWReq *io, UBYTE eptype,
                     BOOL in, BOOL split)
{
    memset(c, 0, sizeof(*c));
    c->addr = (UBYTE)io->iouh_DevAddr;
    c->ep = (UBYTE)io->iouh_Endpoint;
    c->eptype = eptype;
    c->in = in;
    c->lowspeed = (io->iouh_Flags & UHFF_LOWSPEED) ? TRUE : FALSE;
    c->mps = io->iouh_MaxPktSize;
    c->split = split;
    c->hub = (UBYTE)io->iouh_SplitHubAddr;
    c->port = (UBYTE)io->iouh_SplitHubPort;
}

static ULONG run(struct chan_req *c, ULONG max_ticks)
{
    ULONG r;
    chan_start(c);
    r = chan_wait(max_ticks);
    if (r & HCINT_SW_TIMEOUT) {
        /* 103.7: the transfer may complete while we halt it; keep that
         * XFERCOMP, otherwise the device's toggle moves on without us
         * (735 toggle errors with a direct low-speed mouse in 103.6) */
        ULONG late = chan_halt();
        if (late & HCINT_XFERCOMP) {
            drv.st.halt_race_ok++;
            r = (r & ~HCINT_SW_TIMEOUT) | late;
        } else
            r |= late & ~HCINT_CHHLTD;
    }
    if (r & HCINT_DATATGLERR)
        drv.st.toggle_err++;
    return r;
}

/* --------------------------------------------- non-periodic split xact */

/*
 * One downstream transaction through the hub's transaction translator.
 * Returns the final HCINT; *resid gets HCTSIZ after the CSPLIT.
 * A device NAK is returned as HCINT_NAK (caller decides).
 */
static ULONG split_np(struct chan_req *c, ULONG *hctsiz_out, BOOL retry_nak)
{
    ULONG r = 0, ss, cs;
    ULONG size_ss = c->hctsiz;

    for (ss = 0; ss < SPLIT_SS_BUDGET; ss++) {
        c->complete = FALSE;
        c->hctsiz = size_ss;
        r = run(c, 8 * 8);
        if (r & (HCINT_SW_TIMEOUT | HCINT_ERRORS))
            break;
        if (r & (HCINT_NAK | HCINT_NYET)) {     /* TT busy: retry SSPLIT */
            drv.st.split_tt_busy++;
            hw_wait_next_tick();
            continue;
        }
        if (!(r & HCINT_ACK))
            break;

        hw_wait_next_tick();
        for (cs = 0; cs < SPLIT_CS_BUDGET_NP; cs++) {
            c->complete = TRUE;
            /* CSPLIT OUT carries no data (Linux: xfer_len = 0) */
            c->hctsiz = c->in ? size_ss : (size_ss & ~0x7FFFFUL);
            r = run(c, 8 * 8);
            if (hctsiz_out)
                *hctsiz_out = chan_hctsiz();
            if (r & (HCINT_SW_TIMEOUT | HCINT_ERRORS))
                goto done;
            if (r & HCINT_XFERCOMP)
                goto done;
            if (r & HCINT_NYET) {
                drv.st.split_cs_nyet++;
                hw_wait_next_tick();
                continue;
            }
            break;                  /* NAK or anything else */
        }
        if (cs < SPLIT_CS_BUDGET_NP && retry_nak && (r & HCINT_NAK) &&
            !(r & (HCINT_ERRORS | HCINT_SW_TIMEOUT | HCINT_XFERCOMP))) {
            /* device NAKed the complete-split: control transfers restart
             * with a fresh start-split (USB68/69) */
            hw_wait_next_tick();
            continue;
        }
        break;
    }
done:
    c->complete = FALSE;
    hw_wr(HCSPLT(0), 0);
    if (r & HCINT_XFERCOMP)
        drv.st.split_ok++;
    else if (r & HCINT_NAK)
        drv.st.split_nak++;
    else if (r & (HCINT_ERRORS | HCINT_SW_TIMEOUT))
        drv.st.split_err++;
    return r;
}

/* ------------------------------------------------------------ control */

static void ctrl_after_success(struct IOUsbHWReq *io)
{
    struct UsbSetupData *s = &io->iouh_SetupData;
    UWORD wValue = (UWORD)((s->wValue >> 8) | (s->wValue << 8));
    UWORD wIndex = (UWORD)((s->wIndex >> 8) | (s->wIndex << 8));

    if (s->bmRequestType == 0x00 && s->bRequest == 0x05)        /* SET_ADDRESS */
        tog_clear_dev((UBYTE)wValue);
    else if (s->bmRequestType == 0x00 && s->bRequest == 0x09)   /* SET_CONFIGURATION */
        tog_clear_dev((UBYTE)io->iouh_DevAddr);
    else if (s->bmRequestType == 0x02 && s->bRequest == 0x01 && wValue == 0)
        /* CLEAR_FEATURE(ENDPOINT_HALT): only this endpoint */
        tog_set((UBYTE)io->iouh_DevAddr, (UBYTE)(wIndex & 0xF),
                (wIndex & 0x80) ? TRUE : FALSE, FALSE);
}

/* one control stage; returns HCINT; for split goes through the TT */
static ULONG ctrl_stage(struct chan_req *c, BOOL split, ULONG max_ticks,
                        ULONG *hctsiz_out)
{
    ULONG r;
    if (split)
        return split_np(c, hctsiz_out, TRUE);
    r = run(c, max_ticks);
    if (hctsiz_out)
        *hctsiz_out = chan_hctsiz();
    return r;
}

static UWORD ctrl_once(struct IOUsbHWReq *io, BOOL split, ULONG *actual,
                       ULONG *hcint_out)
{
    struct UsbSetupData *s = &io->iouh_SetupData;
    struct chan_req c;
    UWORD wLength = (UWORD)((s->wLength >> 8) | (s->wLength << 8));
    BOOL data_in = (s->bmRequestType & 0x80) ? TRUE : FALSE;
    ULONG len = wLength, mps, r, hts = 0, bus, tmo;
    BOOL to_dev;

    *actual = 0;
    mps = io->iouh_MaxPktSize ? io->iouh_MaxPktSize : 64;
    if (len > io->iouh_Length)
        len = io->iouh_Length;
    if (len > CHAN_MAX_BYTES - 1023)
        len = CHAN_MAX_BYTES - 1023;
    tmo = CTRL_TIMEOUT_MS * hw_ticks_per_ms();
    if ((io->iouh_Flags & UHFF_NAKTIMEOUT) && io->iouh_NakTimeout)
        tmo = io->iouh_NakTimeout * hw_ticks_per_ms();

    /* SETUP: the 8 bytes are already in USB (little-endian) order */
    memcpy(drv.setup_buf, s, 8);
    if (!hw_dma_pre(drv.setup_buf, 8, TRUE, &bus))
        return UHIOERR_HOSTERROR;
    base_req(&c, io, EPTYPE_CONTROL, FALSE, split);
    c.mps = (UWORD)mps;
    c.hctsiz = 8 | HCTSIZ_PKTCNT(1) | HCTSIZ_PID_SETUP;
    c.bus = bus;
    r = ctrl_stage(&c, split, tmo, NULL);
    hw_dma_post(drv.setup_buf, 8, TRUE);
    *hcint_out = r;
    if (!(r & HCINT_XFERCOMP))
        return map_hcint_error(r);

    /* DATA stage */
    if (len) {
        if (data_in) {
            ULONG done = 0, pid = HCTSIZ_PID_DATA1;
            if (!split) {
                ULONG pk = (len + mps - 1) / mps, resv;
                if (pk > CHAN_MAX_PKTS)
                    pk = CHAN_MAX_PKTS;
                resv = pk * mps;
                if (!hw_dma_pre(drv.data_buf, resv, FALSE, &bus))
                    return UHIOERR_HOSTERROR;
                base_req(&c, io, EPTYPE_CONTROL, TRUE, FALSE);
                c.mps = (UWORD)mps;
                c.hctsiz = resv | HCTSIZ_PKTCNT(pk) | HCTSIZ_PID_DATA1;
                c.bus = bus;
                r = ctrl_stage(&c, FALSE, tmo, &hts);
                hw_dma_post(drv.data_buf, resv, FALSE);
                *hcint_out = r;
                if (!(r & HCINT_XFERCOMP))
                    return map_hcint_error(r);
                done = resv - HCTSIZ_XFERSIZE(hts);
                if (done > len)
                    done = len;
            } else {
                /* one TT transaction per max packet; full-MPS reservation
                 * even for the tail (USB69 / Linux split-IN rule) */
                while (done < len) {
                    ULONG got;
                    if (!hw_dma_pre(drv.data_buf + done, mps, FALSE, &bus))
                        return UHIOERR_HOSTERROR;
                    base_req(&c, io, EPTYPE_CONTROL, TRUE, TRUE);
                    c.mps = (UWORD)mps;
                    c.hctsiz = mps | HCTSIZ_PKTCNT(1) | pid;
                    c.bus = bus;
                    r = split_np(&c, &hts, TRUE);
                    hw_dma_post(drv.data_buf + done, mps, FALSE);
                    *hcint_out = r;
                    if (!(r & HCINT_XFERCOMP))
                        return map_hcint_error(r);
                    got = mps - HCTSIZ_XFERSIZE(hts);
                    if (got > len - done)
                        got = len - done;
                    done += got;
                    pid = (pid == HCTSIZ_PID_DATA1) ? HCTSIZ_PID_DATA0
                                                    : HCTSIZ_PID_DATA1;
                    if (got < mps)
                        break;              /* short packet ends the stage */
                }
            }
            CopyMem(drv.data_buf, io->iouh_Data, done);
            *actual = done;
        } else {
            ULONG done = 0, pid = HCTSIZ_PID_DATA1;
            CopyMem(io->iouh_Data, drv.data_buf, len);
            if (!split) {
                ULONG pk = (len + mps - 1) / mps;
                if (!hw_dma_pre(drv.data_buf, len, TRUE, &bus))
                    return UHIOERR_HOSTERROR;
                base_req(&c, io, EPTYPE_CONTROL, FALSE, FALSE);
                c.mps = (UWORD)mps;
                c.hctsiz = len | HCTSIZ_PKTCNT(pk) | HCTSIZ_PID_DATA1;
                c.bus = bus;
                r = ctrl_stage(&c, FALSE, tmo, NULL);
                hw_dma_post(drv.data_buf, len, TRUE);
                *hcint_out = r;
                if (!(r & HCINT_XFERCOMP))
                    return map_hcint_error(r);
                done = len;
            } else {
                while (done < len) {
                    ULONG n = len - done;
                    if (n > mps)
                        n = mps;
                    if (!hw_dma_pre(drv.data_buf + done, n, TRUE, &bus))
                        return UHIOERR_HOSTERROR;
                    base_req(&c, io, EPTYPE_CONTROL, FALSE, TRUE);
                    c.mps = (UWORD)mps;
                    c.hctsiz = n | HCTSIZ_PKTCNT(1) | pid;
                    c.bus = bus;
                    r = split_np(&c, NULL, TRUE);
                    hw_dma_post(drv.data_buf + done, n, TRUE);
                    *hcint_out = r;
                    if (!(r & HCINT_XFERCOMP))
                        return map_hcint_error(r);
                    done += n;
                    pid = (pid == HCTSIZ_PID_DATA1) ? HCTSIZ_PID_DATA0
                                                    : HCTSIZ_PID_DATA1;
                }
            }
            *actual = done;
        }
    }

    /* STATUS stage: opposite direction, zero length, DATA1 */
    {
        BOOL st_in = (len == 0 || !data_in);
        ULONG resv = st_in && split ? mps : 0;
        to_dev = !st_in;
        if (!hw_dma_pre(drv.status_buf, resv ? resv : 4, to_dev, &bus))
            return UHIOERR_HOSTERROR;
        base_req(&c, io, EPTYPE_CONTROL, st_in, split);
        c.mps = (UWORD)mps;
        c.hctsiz = resv | HCTSIZ_PKTCNT(1) | HCTSIZ_PID_DATA1;
        c.bus = bus;
        r = ctrl_stage(&c, split, tmo, NULL);
        hw_dma_post(drv.status_buf, resv ? resv : 4, to_dev);
        *hcint_out = r;
        if (!(r & HCINT_XFERCOMP))
            return map_hcint_error(r);
    }
    return UHIOERR_NO_ERROR;
}

void xfer_control(struct IOUsbHWReq *io)
{
    BOOL split = want_split(io);
    ULONG actual = 0, hcint = 0;
    UWORD err = UHIOERR_HOSTERROR;
    int tries;

    io->iouh_Actual = 0;
    if (io->iouh_Dir != UHDIR_SETUP || io->iouh_DevAddr > 127) {
        set_err(io, UHIOERR_BADPARAMS, 0);
        return;
    }
    if (ls_behind_fs_hub(io, split)) {
        set_err(io, UHIOERR_HOSTERROR, 0x7FF0);
        return;
    }
    {
        UWORD wl = (UWORD)((io->iouh_SetupData.wLength >> 8) |
                           (io->iouh_SetupData.wLength << 8));
        if (wl && !io->iouh_Data) {
            set_err(io, UHIOERR_BADPARAMS, 0);
            return;
        }
    }

    for (tries = 0; tries < XACT_RETRIES; tries++) {
        err = ctrl_once(io, split, &actual, &hcint);
        if (err != UHIOERR_TIMEOUT || !(hcint & HCINT_XACTERR))
            break;
        drv.st.xact_retry++;
        hw_wait_next_tick();
    }
    io->iouh_Actual = actual;
    if (err == UHIOERR_NO_ERROR) {
        io->iouh_Req.io_Error = 0;
        io->iouh_ExtError = 0;
        ctrl_after_success(io);
    } else {
        set_err(io, err, hcint);
    }
}

/* --------------------------------------------------------------- bulk */

/*
 * Runs one time slice of a bulk request.  Returns TRUE when the request is
 * finished (io_Error set), FALSE when it should be continued later.
 */
BOOL xfer_bulk_step(struct IOUsbHWReq *io)
{
    BOOL in = (io->iouh_Dir == UHDIR_IN);
    BOOL split;
    UBYTE addr = (UBYTE)io->iouh_DevAddr, ep = (UBYTE)io->iouh_Endpoint;
    ULONG mps = io->iouh_MaxPktSize, remaining, chunk, pk, bus, r, hts, n;
    ULONG xfersize;
    struct chan_req c;
    UBYTE *ubuf = (UBYTE *)io->iouh_Data;

    if ((io->iouh_Dir != UHDIR_IN && io->iouh_Dir != UHDIR_OUT) ||
        mps == 0 || mps > 1024 || io->iouh_DevAddr > 127 ||
        (io->iouh_Length && !io->iouh_Data)) {
        set_err(io, UHIOERR_BADPARAMS, 0);
        return TRUE;
    }
    split = want_split(io);
    if (ls_behind_fs_hub(io, split)) {
        set_err(io, UHIOERR_HOSTERROR, 0x7FF0);
        return TRUE;
    }

    if (REQ_START(io) == 0)
        nak_progress(io);

    remaining = io->iouh_Length - io->iouh_Actual;

    if (split) {
        /* one packet per TT transaction */
        chunk = remaining > mps ? mps : remaining;
        if (in) {
            if (!hw_dma_pre(drv.data_buf, mps, FALSE, &bus)) goto dma_fail;
        } else {
            CopyMem(ubuf + io->iouh_Actual, drv.data_buf, chunk);
            if (!hw_dma_pre(drv.data_buf, chunk ? chunk : 4, TRUE, &bus)) goto dma_fail;
        }
        base_req(&c, io, EPTYPE_BULK, in, TRUE);
        c.hctsiz = (in ? mps : chunk) | HCTSIZ_PKTCNT(1) |
                   pid_of(tog_get(addr, ep, in));
        c.bus = bus;
        hts = 0;
        r = split_np(&c, &hts, FALSE);
        hw_dma_post(drv.data_buf, in ? mps : (chunk ? chunk : 4), !in);
        if (r & HCINT_XFERCOMP) {
            tog_set(addr, ep, in, !tog_get(addr, ep, in));
            n = in ? mps - HCTSIZ_XFERSIZE(hts) : chunk;
            if (in) {
                if (n > remaining) n = remaining;
                CopyMem(drv.data_buf, ubuf + io->iouh_Actual, n);
            }
            io->iouh_Actual += n;
            nak_progress(io);
            if ((in && n < mps) || io->iouh_Actual >= io->iouh_Length)
                goto finished;
            return FALSE;
        }
        if (r & HCINT_NAK)
            goto nak;
        set_err(io, map_hcint_error(r), r);
        return TRUE;
    }

    /* direct: up to 65535 bytes / 511 packets per channel run (Linux BCM) */
    chunk = remaining;
    {
        ULONG maxc = (CHAN_MAX_BYTES / mps) * mps;
        if (maxc > CHAN_MAX_PKTS * mps)
            maxc = CHAN_MAX_PKTS * mps;
        if (chunk > maxc)
            chunk = maxc;
    }
    pk = chunk ? (chunk + mps - 1) / mps : 1;
    xfersize = in ? pk * mps : chunk;

    if (in) {
        if (!hw_dma_pre(drv.data_buf, xfersize, FALSE, &bus)) goto dma_fail;
    } else {
        if (chunk)
            CopyMem(ubuf + io->iouh_Actual, drv.data_buf, chunk);
        if (!hw_dma_pre(drv.data_buf, chunk ? chunk : 4, TRUE, &bus)) goto dma_fail;
    }

    base_req(&c, io, EPTYPE_BULK, in, FALSE);
    c.hctsiz = xfersize | HCTSIZ_PKTCNT(pk) | pid_of(tog_get(addr, ep, in));
    if (!in && (REQ_DUE(io) & DUE_PING) && drv.port_speed == 0)
        c.hctsiz |= HCTSIZ_DOPNG;
    c.bus = bus;

    /* time slice: 4 ms while data flows, a short probe (2 (micro)frames)
     * for a device that only NAKs (idle network adapter), then give
     * interrupt polls and other devices a turn */
    r = run(&c, (REQ_DUE(io) & DUE_ACTIVE) ? 4 * hw_ticks_per_ms() : 2);
    hts = chan_hctsiz();
    hw_dma_post(drv.data_buf, in ? xfersize : (chunk ? chunk : 4), !in);
    drv.st.bulk_slices++;

    /* next PID always from the controller */
    tog_set(addr, ep, in, pid_is_data1(hts));

    if (r & HCINT_XFERCOMP) {
        n = in ? xfersize - HCTSIZ_XFERSIZE(hts) : chunk;
        if (n > chunk) n = chunk;
    } else {
        /* halted early: whole packets only (Linux get_actual_xfer_length) */
        ULONG left = HCTSIZ_GET_PKTCNT(hts);
        n = (left <= pk) ? (pk - left) * mps : 0;
        if (n > chunk) n = chunk;
    }

    if (in && n)
        CopyMem(drv.data_buf, ubuf + io->iouh_Actual, n);
    io->iouh_Actual += n;
    drv.st.bulk_bytes += n;

    if (n) {
        nak_progress(io);
        REQ_DUE(io) |= DUE_ACTIVE;
    } else
        REQ_DUE(io) &= ~DUE_ACTIVE;

    if (!in) {
        if (r & HCINT_NYET)
            REQ_DUE(io) |= DUE_PING;
        else if (r & HCINT_XFERCOMP)
            REQ_DUE(io) &= ~DUE_PING;
    }

    if (r & HCINT_XFERCOMP) {
        if (in && n < xfersize && n < chunk)
            goto finished;                  /* short packet */
        if (io->iouh_Actual >= io->iouh_Length)
            goto finished;
        return FALSE;
    }
    if (r & HCINT_STALL) {
        tog_set(addr, ep, in, FALSE);
        set_err(io, UHIOERR_STALL, r);
        return TRUE;
    }
    if (r & (HCINT_AHBERR | HCINT_BBLERR | HCINT_DATATGLERR)) {
        set_err(io, map_hcint_error(r), r);
        return TRUE;
    }
    if (r & HCINT_XACTERR) {
        ULONG cnt = ((REQ_DUE(io) >> 16) & 0xFF) + 1;
        REQ_DUE(io) = (REQ_DUE(io) & ~0x00FF0000UL) | (cnt << 16);
        drv.st.xact_retry++;
        if (cnt >= XACT_RETRIES) {
            set_err(io, UHIOERR_TIMEOUT, r);
            return TRUE;
        }
        return FALSE;
    }
    if (n)
        REQ_DUE(io) &= ~0x00FF0000UL;       /* progress resets error count */
    /* NAK / NYET / slice ended: continue later */
nak:
    if (nak_expired(io)) {
        set_err(io, UHIOERR_NAKTIMEOUT, r);
        return TRUE;
    }
    return FALSE;

finished:
    io->iouh_Req.io_Error = 0;
    io->iouh_ExtError = 0;
    return TRUE;

dma_fail:
    set_err(io, UHIOERR_HOSTERROR, 0x7FFE);
    return TRUE;
}

/* ---------------------------------------------------------- interrupt */

/* REQ_DUE for interrupt requests: bit 24 valid, bits 16-23 consecutive
 * error count, bits 0-13 frame number of the next allowed poll. */
BOOL xfer_intr_due(struct IOUsbHWReq *io, UWORD now)
{
    ULONG d = REQ_DUE(io);
    if (!(d & DUE_VALID))
        return TRUE;
    /* due when 'now' has reached the stored frame (14-bit wrap-around) */
    return hw_frame_diff(now, (UWORD)(d & 0x3FFF)) < 0x2000;
}

static ULONG intr_interval_ticks(struct IOUsbHWReq *io)
{
    ULONG iv = io->iouh_Interval ? io->iouh_Interval : 1;
    ULONG t;
    if (io->iouh_Flags & UHFF_HIGHSPEED)
        t = iv;                                 /* already in microframes */
    else
        t = iv * hw_ticks_per_ms();             /* ms */
    if (t > 0x1000)
        t = 0x1000;
    return t ? t : 1;
}

/* Periodic split IN/OUT: SSPLIT, then up to three CSPLIT slots. */
static ULONG split_periodic(struct chan_req *c, ULONG *hts_out)
{
    ULONG r = 0, size_ss = c->hctsiz;
    int err, cs;
    ULONG n;
    UWORD ss_frame = 0;

    for (err = 0; err < XACT_RETRIES; err++) {
        /* start-split only early in the frame, so the complete-splits
         * stay inside the same 1 ms frame (Linux uframe scheduling) */
        /* 103.5: arm only in microframe 7 or 0, so the start-split is on
         * the wire in microframe 0 or 1 and up to six complete-splits fit
         * into the same frame (103.4: 7,0,1,2 -> 5 % NYET give-ups) */
        for (n = 0; n < 16; n++) {
            UWORD uf = hw_frnum() & 7;
            if (uf == 7 || uf == 0)
                break;
            hw_wait_next_tick();
        }
        c->complete = FALSE;
        c->hctsiz = size_ss;
        c->periodic_next = TRUE;
        r = run(c, 8);
        if (r & HCINT_STALL)
            goto out;
        if (r & (HCINT_XACTERR | HCINT_FRMOVRUN | HCINT_SW_TIMEOUT)) {
            drv.st.split_xacterr++;
            continue;
        }
        if (r & HCINT_ERRORS)
            goto out;
        if (r & (HCINT_NAK | HCINT_NYET)) {     /* TT refused the start */
            drv.st.split_tt_busy++;
            r = HCINT_NAK | HCINT_CHHLTD;
            goto out;
        }
        if (!(r & HCINT_ACK))
            goto out;

        ss_frame = (UWORD)(hw_frnum() >> 3);
        hw_wait_next_tick();
        for (cs = 0; cs < SPLIT_CS_TRIES_P; cs++) {
            /* Linux dwc2_hc_nyet_intr: keep doing complete-splits until
             * the full frame of the start-split is over ("past_end") */
            if (cs >= 3 && (UWORD)(hw_frnum() >> 3) != ss_frame)
                break;
            c->complete = TRUE;
            c->hctsiz = c->in ? size_ss : (size_ss & ~0x7FFFFUL);
            c->periodic_next = TRUE;
            r = run(c, 8);
            if (hts_out)
                *hts_out = chan_hctsiz();
            if (r & HCINT_XFERCOMP) {
                if (cs >= 3)
                    drv.st.split_cs_late_ok++;
                goto out;
            }
            if (r & HCINT_STALL)
                goto out;
            if (r & (HCINT_XACTERR | HCINT_FRMOVRUN | HCINT_SW_TIMEOUT)) {
                drv.st.split_xacterr++;
                break;                          /* retry from SSPLIT */
            }
            if (r & HCINT_ERRORS)
                goto out;                       /* incl. toggle mismatch */
            if (r & HCINT_NYET) {
                drv.st.split_cs_nyet++;
                if (cs + 1 < SPLIT_CS_TRIES_P)
                    hw_wait_next_tick();
                continue;
            }
            /* NAK: the device has nothing to report */
            r = HCINT_NAK | HCINT_CHHLTD;
            goto out;
        }
        if (cs == SPLIT_CS_TRIES_P || (cs >= 3 && (r & HCINT_NYET))) {
            /* all CSPLITs NYET: not an error (Linux "past_end") */
            drv.st.split_nyet_giveup++;
            r = HCINT_NAK | HCINT_CHHLTD;
            goto out;
        }
    }
    drv.st.split_err++;
out:
    c->complete = FALSE;
    hw_wr(HCSPLT(0), 0);
    if (r & HCINT_XFERCOMP)
        drv.st.split_ok++;
    else if (r & HCINT_NAK)
        drv.st.split_nak++;
    return r;
}

/*
 * One poll of an interrupt endpoint.  Returns TRUE when the request is
 * finished.  A NAK keeps it pending until the next interval.
 */
BOOL xfer_intr_poll(struct IOUsbHWReq *io)
{
    BOOL in = (io->iouh_Dir == UHDIR_IN);
    BOOL split;
    UBYTE addr = (UBYTE)io->iouh_DevAddr, ep = (UBYTE)io->iouh_Endpoint;
    ULONG mps = io->iouh_MaxPktSize, len, bus, r, hts = 0, n;
    struct chan_req c;

    if ((io->iouh_Dir != UHDIR_IN && io->iouh_Dir != UHDIR_OUT) ||
        mps == 0 || mps > 1024 || io->iouh_DevAddr > 127 ||
        io->iouh_Length == 0 || !io->iouh_Data) {
        set_err(io, UHIOERR_BADPARAMS, 0);
        return TRUE;
    }
    split = want_split(io);
    if (ls_behind_fs_hub(io, split)) {
        set_err(io, UHIOERR_HOSTERROR, 0x7FF0);
        return TRUE;
    }
    if (REQ_START(io) == 0)
        nak_progress(io);
    drv.st.intr_polls++;

    /* one packet per request (USB57 behaviour, safe for HID) */
    len = io->iouh_Length > mps ? mps : io->iouh_Length;

    if (in) {
        if (!hw_dma_pre(drv.intr_buf, mps, FALSE, &bus)) goto dma_fail;
    } else {
        CopyMem(io->iouh_Data, drv.intr_buf, len);
        if (!hw_dma_pre(drv.intr_buf, len, TRUE, &bus)) goto dma_fail;
    }

    base_req(&c, io, EPTYPE_INTR, in, split);
    c.hctsiz = (in ? mps : len) | HCTSIZ_PKTCNT(1) |
               pid_of(tog_get(addr, ep, in));
    c.bus = bus;
    c.periodic_next = TRUE;

    if (split)
        r = split_periodic(&c, &hts);
    else {
        r = run(&c, 3 * hw_ticks_per_ms() + 2);
        hts = chan_hctsiz();
    }
    hw_dma_post(drv.intr_buf, in ? mps : len, !in);

    if (r & HCINT_XFERCOMP) {
        if (split)
            tog_set(addr, ep, in, !tog_get(addr, ep, in));
        else
            tog_set(addr, ep, in, pid_is_data1(hts));
        n = in ? mps - HCTSIZ_XFERSIZE(hts) : len;
        if (n > len)
            n = len;
        if (in && n)
            CopyMem(drv.intr_buf, io->iouh_Data, n);
        io->iouh_Actual = n;
        io->iouh_Req.io_Error = 0;
        io->iouh_ExtError = 0;
        REQ_DUE(io) = 0;
        drv.st.intr_data++;
        return TRUE;
    }
    if (r & HCINT_STALL) {
        tog_set(addr, ep, in, FALSE);
        set_err(io, UHIOERR_STALL, r);
        return TRUE;
    }
    /* 103.6: toggle mismatch on IN is counted (tgl_in_*) but handled like
     * in 103.1-103.3 (wait one interval).  The immediate re-poll of
     * 103.4/103.5 broke a directly attached low-speed mouse. */
    if (in && (r & HCINT_DATATGLERR)) {
        if (split)
            drv.st.tgl_in_split++;
        else
            drv.st.tgl_in_direct++;
    }
    if (!(r & HCINT_NAK) && (r & HCINT_ERRORS)) {
        /* count consecutive transaction errors; tolerate a few */
        ULONG cnt = ((REQ_DUE(io) >> 16) & 0xFF) + 1;
        drv.st.intr_err++;
        if (cnt >= XACT_RETRIES) {
            set_err(io, map_hcint_error(r), r);
            return TRUE;
        }
        REQ_DUE(io) = DUE_VALID | (cnt << 16) |
                      ((hw_frnum() + intr_interval_ticks(io)) & 0x3FFF);
        return FALSE;
    }
    /* SW timeout alone: we halted a periodic poll that got no answer in
     * time - same as a NAK */

    /* NAK: nothing new; poll again after the interval */
    drv.st.intr_nak++;
    if (nak_expired(io)) {
        set_err(io, UHIOERR_NAKTIMEOUT, r);
        return TRUE;
    }
    REQ_DUE(io) = DUE_VALID |
                  ((hw_frnum() + intr_interval_ticks(io)) & 0x3FFF);
    return FALSE;

dma_fail:
    set_err(io, UHIOERR_HOSTERROR, 0x7FFE);
    return TRUE;
}

/* ===================================================================
 * Periodic engine (105.0)
 *
 * Interrupt-endpoint polls (mouse, keyboard, hub status) run on DWC2
 * channel 1, completely inside the interrupt:
 *   - system timer C1 fires when the next poll is due (or when the
 *     microframe for a start-split has come),
 *   - the channel-halted interrupt of channel 1 advances the state
 *     machine (SSPLIT -> CSPLIT ... -> result),
 *   - the request is replied from the interrupt (ReplyMsg is allowed).
 * The unit task keeps control and bulk transfers on channel 0.  So the
 * polls no longer wait for the task, which higher-priority tasks
 * (Intuition, file systems) could hold up for 30-70 ms.
 *
 * The decisions per transaction are the same as split_periodic() and
 * xfer_intr_poll() above (Linux dwc2 hcd_intr.c rules), only event
 * driven instead of waiting.
 *
 * Every function here runs with interrupts disabled (from the level 6
 * server, or from the task inside Disable()).
 * =================================================================== */

#define PE_IDLE     0
#define PE_DIRECT   1
#define PE_SS       2
#define PE_CS       3
#define PE_WAIT_UF  4

#ifdef ROMBUILD
#define pe RV(pe)
#else
volatile BOOL pe_enabled;

static struct PeState pe;
#endif

static ULONG pe_ms(void)
{
    return irq_us_now() / 1000UL;
}

static void pe_reply(struct IOUsbHWReq *io)
{
    Remove(&io->iouh_Req.io_Message.mn_Node);
    ReplyMsg(&io->iouh_Req.io_Message);
}

/* 105.1: waiting for microframe 7.  105.0 computed the time from
 * HFNUM.FRREM/HFIR and arrived too early about 4 times per poll (FRREM
 * and HFIR do not use the same unit here).  Now: from microframe 1..5
 * sleep whole microframes so we land somewhere inside microframe 6,
 * then busy-wait the rest (at most 125 us) inside the interrupt. */
static void pe_ss_start(void)
{
    UWORD uf = hw_frnum() & 7;
    ULONG n;

    if (uf != 7 && uf != 0 && uf != 6) {
        pe.phase = PE_WAIT_UF;
        drv.st.pe_uf_wait++;
        irq_c1_at(((6 - uf) & 7) * 125UL);
        return;
    }
    if (uf == 6) {
        for (n = 0; n < 0x10000UL; n++)
            if ((hw_frnum() & 7) != 6)
                break;
    }
    pe.c.complete = FALSE;
    pe.c.hctsiz = pe.size_ss;
    pe.c.periodic_next = TRUE;
    pe.phase = PE_SS;
    chan1_start(&pe.c);
}

static void pe_cs_start(void)
{
    pe.c.complete = TRUE;
    pe.c.hctsiz = pe.in ? pe.size_ss : (pe.size_ss & ~0x7FFFFUL);
    pe.c.periodic_next = TRUE;
    pe.phase = PE_CS;
    chan1_start(&pe.c);
}

/* the poll is over: decide like xfer_intr_poll() */
static void pe_done(ULONG r, ULONG hts)
{
    struct IOUsbHWReq *io = pe.cur;
    UBYTE addr = (UBYTE)io->iouh_DevAddr, ep = (UBYTE)io->iouh_Endpoint;
    BOOL in = pe.in, split = pe.split, finished = FALSE;
    ULONG n;

    hw_wr(HCSPLT(1), 0);
    hw_dma_post(drv.intr_buf, in ? pe.mps : pe.len, !in);
    pe.cur = NULL;
    pe.phase = PE_IDLE;

    if (split) {
        if (r & HCINT_XFERCOMP)
            drv.st.split_ok++;
        else if (r & HCINT_NAK)
            drv.st.split_nak++;
    }

    if (r & HCINT_XFERCOMP) {
        if (split)
            tog_set(addr, ep, in, !tog_get(addr, ep, in));
        else
            tog_set(addr, ep, in, pid_is_data1(hts));
        n = in ? pe.mps - HCTSIZ_XFERSIZE(hts) : pe.len;
        if (n > pe.len)
            n = pe.len;
        if (in && n)
            CopyMem(drv.intr_buf, io->iouh_Data, n);
        io->iouh_Actual = n;
        io->iouh_Req.io_Error = 0;
        io->iouh_ExtError = 0;
        REQ_DUE(io) = 0;
        drv.st.intr_data++;
        finished = TRUE;
    } else if (r & HCINT_STALL) {
        tog_set(addr, ep, in, FALSE);
        set_err(io, UHIOERR_STALL, r);
        finished = TRUE;
    } else {
        if (in && (r & HCINT_DATATGLERR)) {
            if (split)
                drv.st.tgl_in_split++;
            else
                drv.st.tgl_in_direct++;
        }
        if (!(r & HCINT_NAK) && (r & HCINT_ERRORS)) {
            ULONG cnt = ((REQ_DUE(io) >> 16) & 0xFF) + 1;
            drv.st.intr_err++;
            if (cnt >= XACT_RETRIES) {
                set_err(io, map_hcint_error(r), r);
                finished = TRUE;
            } else
                REQ_DUE(io) = DUE_VALID | (cnt << 16) |
                              ((hw_frnum() + intr_interval_ticks(io)) & 0x3FFF);
        } else {
            drv.st.intr_nak++;
            if ((io->iouh_Flags & UHFF_NAKTIMEOUT) && io->iouh_NakTimeout &&
                (pe_ms() + 1 - REQ_START(io)) >= io->iouh_NakTimeout) {
                set_err(io, UHIOERR_NAKTIMEOUT, r);
                finished = TRUE;
            } else
                REQ_DUE(io) = DUE_VALID |
                              ((hw_frnum() + intr_interval_ticks(io)) & 0x3FFF);
        }
    }
    if (finished)
        pe_reply(io);
}

/* start one poll; FALSE if the request was finished right away */
static BOOL pe_start(struct IOUsbHWReq *io, UWORD now)
{
    BOOL in = (io->iouh_Dir == UHDIR_IN);
    ULONG mps = io->iouh_MaxPktSize, bus;

    if ((io->iouh_Dir != UHDIR_IN && io->iouh_Dir != UHDIR_OUT) ||
        mps == 0 || mps > 1024 || io->iouh_DevAddr > 127 ||
        io->iouh_Length == 0 || !io->iouh_Data) {
        set_err(io, UHIOERR_BADPARAMS, 0);
        pe_reply(io);
        return FALSE;
    }
    pe.split = want_split(io);
    if (ls_behind_fs_hub(io, pe.split)) {
        set_err(io, UHIOERR_HOSTERROR, 0x7FF0);
        pe_reply(io);
        return FALSE;
    }
    if (REQ_START(io) == 0)
        REQ_START(io) = pe_ms() + 1;

    if (REQ_DUE(io) & DUE_VALID) {
        ULONG late = hw_frame_diff(now, (UWORD)(REQ_DUE(io) & DUE_FRAME_MASK))
                     / hw_ticks_per_ms();
        int b = late < 1 ? 0 : late < 2 ? 1 : late < 4 ? 2 :
                late < 8 ? 3 : late < 16 ? 4 : 5;
        drv.st.late[b]++;
        if (late > drv.st.late_max_ms)
            drv.st.late_max_ms = late;
    }
    drv.st.intr_polls++;

    pe.in = in;
    pe.mps = mps;
    pe.len = io->iouh_Length > mps ? mps : io->iouh_Length;
    if (in) {
        if (!hw_dma_pre(drv.intr_buf, mps, FALSE, &bus))
            goto dma_fail;
    } else {
        CopyMem(io->iouh_Data, drv.intr_buf, pe.len);
        if (!hw_dma_pre(drv.intr_buf, pe.len, TRUE, &bus))
            goto dma_fail;
    }
    base_req(&pe.c, io, EPTYPE_INTR, in, pe.split);
    pe.c.hctsiz = (in ? mps : pe.len) | HCTSIZ_PKTCNT(1) |
                  pid_of(tog_get((UBYTE)io->iouh_DevAddr, (UBYTE)io->iouh_Endpoint, in));
    pe.c.bus = bus;
    pe.c.periodic_next = TRUE;
    pe.size_ss = pe.c.hctsiz;
    pe.cur = io;
    pe.abort = FALSE;
    pe.errs = 0;
    pe.cs = 0;
    pe.start_us = irq_us_now();

    if (pe.split)
        pe_ss_start();
    else {
        pe.phase = PE_DIRECT;
        chan1_start(&pe.c);
    }
    return TRUE;

dma_fail:
    set_err(io, UHIOERR_HOSTERROR, 0x7FFE);
    pe_reply(io);
    return FALSE;
}

/* start the next due poll, or arm the timer for the earliest one */
void pe_kick(void)
{
    struct Node *n;

    if (!pe_enabled || pe.cur)
        return;
    for (;;) {
        UWORD now = hw_frnum();
        struct IOUsbHWReq *due = NULL;
        ULONG min_ticks = 0xFFFF;

        for (n = drv.intr_q.lh_Head; n->ln_Succ; n = n->ln_Succ) {
            struct IOUsbHWReq *r = (struct IOUsbHWReq *)n;
            if (xfer_intr_due(r, now)) {
                due = r;
                break;
            } else {
                ULONG t = hw_frame_diff((UWORD)(REQ_DUE(r) & DUE_FRAME_MASK), now);
                if (t < min_ticks)
                    min_ticks = t;
            }
        }
        if (due) {
            /* rotate, so every endpoint gets its turn */
            Remove(&due->iouh_Req.io_Message.mn_Node);
            AddTail(&drv.intr_q, &due->iouh_Req.io_Message.mn_Node);
            if (pe_start(due, now))
                return;
            continue;                   /* finished at once: next one */
        }
        if (min_ticks != 0xFFFF)
            irq_c1_at(min_ticks * (1000UL / hw_ticks_per_ms()));
        return;
    }
}

void pe_on_timer(void)
{
    drv.st.pe_timer_irq++;
    if (pe.cur && pe.phase == PE_WAIT_UF)
        pe_ss_start();
    else if (!pe.cur)
        pe_kick();
}

void pe_on_chan(ULONG r)
{
    ULONG hts;

    drv.st.pe_chan_irq++;
    if (!pe.cur || pe.phase == PE_IDLE || pe.phase == PE_WAIT_UF) {
        drv.st.pe_spurious++;
        return;
    }
    if (pe.abort) {
        struct IOUsbHWReq *io = pe.cur;
        hw_wr(HCSPLT(1), 0);
        hw_dma_post(drv.intr_buf, pe.in ? pe.mps : pe.len, !pe.in);
        pe.cur = NULL;
        pe.phase = PE_IDLE;
        io->iouh_Req.io_Error = IOERR_ABORTED;
        pe_reply(io);
        pe_kick();
        return;
    }
    if (r & HCINT_DATATGLERR)
        drv.st.toggle_err++;

    switch (pe.phase) {
    case PE_DIRECT:
        pe_done(r, chan1_hctsiz());
        break;

    case PE_SS:
        if (r & HCINT_STALL) {
            pe_done(r, 0);
        } else if (r & (HCINT_XACTERR | HCINT_FRMOVRUN)) {
            drv.st.split_xacterr++;
            if (++pe.errs < XACT_RETRIES) {
                pe_ss_start();
                return;
            }
            drv.st.split_err++;
            pe_done(r, 0);
        } else if (r & HCINT_ERRORS) {
            pe_done(r, 0);
        } else if (r & (HCINT_NAK | HCINT_NYET)) {
            drv.st.split_tt_busy++;     /* TT refused the start */
            pe_done(HCINT_NAK | HCINT_CHHLTD, 0);
        } else if (!(r & HCINT_ACK)) {
            pe_done(r, 0);
        } else {
            pe.ss_frame = (UWORD)(hw_frnum() >> 3);
            pe.cs = 0;
            pe_cs_start();
            return;
        }
        break;

    case PE_CS:
        hts = chan1_hctsiz();
        if (r & HCINT_XFERCOMP) {
            if (pe.cs >= 3)
                drv.st.split_cs_late_ok++;
            pe_done(r, hts);
        } else if (r & HCINT_STALL) {
            pe_done(r, hts);
        } else if (r & (HCINT_XACTERR | HCINT_FRMOVRUN)) {
            drv.st.split_xacterr++;
            if (++pe.errs < XACT_RETRIES) {
                pe_ss_start();          /* retry from the start-split */
                return;
            }
            drv.st.split_err++;
            pe_done(r, hts);
        } else if (r & HCINT_ERRORS) {
            pe_done(r, hts);
        } else if (r & HCINT_NYET) {
            drv.st.split_cs_nyet++;
            pe.cs++;
            if (pe.cs >= SPLIT_CS_TRIES_P ||
                (pe.cs >= 3 && (UWORD)(hw_frnum() >> 3) != pe.ss_frame)) {
                drv.st.split_nyet_giveup++;     /* Linux "past_end" */
                pe_done(HCINT_NAK | HCINT_CHHLTD, hts);
            } else {
                pe_cs_start();
                return;
            }
        } else {
            pe_done(HCINT_NAK | HCINT_CHHLTD, hts);   /* device NAK */
        }
        break;
    }
    if (!pe.cur)
        pe_kick();
}

BOOL pe_is_current(struct IOUsbHWReq *io)
{
    return io == pe.cur;
}

/* AbortIO (interrupts disabled by the caller) */
BOOL pe_abort(struct IOUsbHWReq *io)
{
    if (io != pe.cur)
        return FALSE;
    if (pe.phase == PE_WAIT_UF) {
        /* channel not started yet */
        hw_dma_post(drv.intr_buf, pe.in ? pe.mps : pe.len, !pe.in);
        pe.cur = NULL;
        pe.phase = PE_IDLE;
        io->iouh_Req.io_Error = IOERR_ABORTED;
        pe_reply(io);
        pe_kick();
    } else {
        pe.abort = TRUE;                /* finished on the halt interrupt */
        chan1_disable();
    }
    return TRUE;
}

/* unit task, every wake-up: free a poll that never finished */
void pe_watchdog(void)
{
    Disable();
    if (pe.cur && (!pe_enabled ||
                   (irq_us_now() - pe.start_us) > 100000UL)) {
        struct IOUsbHWReq *io = pe.cur;
        drv.st.pe_stuck++;
        chan1_disable();
        hw_wr(HCINT(1), 0xFFFFFFFFUL);
        hw_wr(HCSPLT(1), 0);
        hw_dma_post(drv.intr_buf, pe.in ? pe.mps : pe.len, !pe.in);
        pe.cur = NULL;
        pe.phase = PE_IDLE;
        if (pe.abort) {
            io->iouh_Req.io_Error = IOERR_ABORTED;
            pe_reply(io);
        } else
            REQ_DUE(io) = DUE_VALID |
                          ((hw_frnum() + intr_interval_ticks(io)) & 0x3FFF);
        pe_kick();
    }
    Enable();
}

/* USB bus reset: stop the engine's running poll */
void pe_reset(void)
{
    Disable();
    if (pe.cur) {
        chan1_disable();
        hw_wr(HCINT(1), 0xFFFFFFFFUL);
        hw_wr(HCSPLT(1), 0);
        hw_dma_post(drv.intr_buf, pe.in ? pe.mps : pe.len, !pe.in);
        REQ_DUE(pe.cur) = 0;
        pe.cur = NULL;
        pe.phase = PE_IDLE;
    }
    Enable();
}
