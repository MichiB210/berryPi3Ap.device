/*
 * berrypi3ap.device - USB host controller driver for PiStorm/Emu68 (Pi 3A+)
 * Copyright (C) 2026 MichiB210
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Virtual one-port USB 2.0 root hub (USB57 design): Poseidon talks to it
 * like to any hub; port status/changes come from the DWC2 HPRT0 register.
 */
#include <string.h>
#include <proto/exec.h>
#include "dev.h"
#include "regs.h"

static const UBYTE rh_dev_desc[18] = {
    0x12, 0x01, 0x00, 0x02, 0x09, 0x00, 0x01, 0x40,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01
};

static const UBYTE rh_cfg_desc[25] = {
    0x09, 0x02, 0x19, 0x00, 0x01, 0x01, 0x00, 0xC0, 0x00,
    0x09, 0x04, 0x00, 0x00, 0x01, 0x09, 0x00, 0x01, 0x00,
    0x07, 0x05, 0x81, 0x03, 0x01, 0x00, 0x0C
};

static const UBYTE rh_hub_desc[9] = {
    0x09, 0x29, 0x01, 0x09, 0x00, 0x01, 0x00, 0x00, 0xFF
};

/* hub port feature selectors / change bits */
#define PF_CONNECTION   0
#define PF_ENABLE       1
#define PF_SUSPEND      2
#define PF_OVERCURRENT  3
#define PF_RESET        4
#define PF_POWER        8
#define PF_LOWSPEED     9
#define PF_C_CONNECTION 16
#define PF_C_ENABLE     17
#define PF_C_SUSPEND    18
#define PF_C_OVERCURRENT 19
#define PF_C_RESET      20

BOOL rh_is_addr(UWORD addr)
{
    return addr == drv.rh_addr;
}

void rh_update_changes(void)
{
    ULONG v = hw_hprt();
    UBYTE conn = (v & HPRT_CONN) ? 1 : 0;

    if (v & HPRT_CONNDET)
        drv.rh_change |= 0x0001;
    if (v & HPRT_ENACHG)
        drv.rh_change |= 0x0002;
    if (v & HPRT_OVRCHG)
        drv.rh_change |= 0x0008;
    if (conn != drv.rh_last_conn) {
        drv.rh_change |= 0x0001;
        drv.rh_last_conn = conn;
    }
}

BOOL rh_intr_try(struct IOUsbHWReq *io)
{
    rh_update_changes();
    if (!drv.rh_change)
        return FALSE;
    ((UBYTE *)io->iouh_Data)[0] = 0x02;     /* bit 1 = port 1 changed */
    io->iouh_Actual = 1;
    io->iouh_Req.io_Error = 0;
    return TRUE;
}

static UWORD port_status(void)
{
    ULONG v = hw_hprt();
    UWORD s = 0;

    if (v & HPRT_CONN)   s |= 1 << PF_CONNECTION;
    if (v & HPRT_ENA)    s |= 1 << PF_ENABLE;
    if (v & HPRT_SUSP)   s |= 1 << PF_SUSPEND;
    if (v & HPRT_OVRCUR) s |= 1 << PF_OVERCURRENT;
    if (v & HPRT_RST)    s |= 1 << PF_RESET;
    if (v & HPRT_PWR)    s |= 1 << PF_POWER;
    if (v & HPRT_CONN) {
        ULONG spd = v & HPRT_SPD_MASK;
        if (spd == HPRT_SPD_LOW)
            s |= 1 << PF_LOWSPEED;
        else if (spd == HPRT_SPD_HIGH)
            s |= 1 << 10;                   /* PORT_HIGH_SPEED */
    }
    return s;
}

static void reply_data(struct IOUsbHWReq *io, const UBYTE *src, ULONG size)
{
    UWORD wLength = (UWORD)((io->iouh_SetupData.wLength >> 8) |
                            (io->iouh_SetupData.wLength << 8));
    ULONG n = size;
    if (n > wLength) n = wLength;
    if (n > io->iouh_Length) n = io->iouh_Length;
    if (n && !io->iouh_Data) {
        io->iouh_Req.io_Error = UHIOERR_BADPARAMS;
        return;
    }
    if (n)
        CopyMem((APTR)src, io->iouh_Data, n);
    io->iouh_Actual = n;
    io->iouh_Req.io_Error = 0;
}

static void stall(struct IOUsbHWReq *io)
{
    io->iouh_Actual = 0;
    io->iouh_Req.io_Error = UHIOERR_STALL;
}

static void ok(struct IOUsbHWReq *io)
{
    io->iouh_Actual = 0;
    io->iouh_Req.io_Error = 0;
}

void rh_control(struct IOUsbHWReq *io)
{
    struct UsbSetupData *s = &io->iouh_SetupData;
    UWORD wValue = (UWORD)((s->wValue >> 8) | (s->wValue << 8));
    UWORD wIndex = (UWORD)((s->wIndex >> 8) | (s->wIndex << 8));
    UBYTE buf[4];

    io->iouh_Actual = 0;

    switch (s->bmRequestType) {
    case 0x00:                                  /* standard, device, OUT */
        switch (s->bRequest) {
        case 0x05:                              /* SET_ADDRESS */
            if (wValue > 127) { stall(io); return; }
            drv.rh_addr = (UBYTE)wValue;
            ok(io); return;
        case 0x09:                              /* SET_CONFIGURATION */
            if (wValue > 1) { stall(io); return; }
            drv.rh_config = (UBYTE)wValue;
            ok(io); return;
        case 0x01:                              /* CLEAR_FEATURE */
        case 0x03:                              /* SET_FEATURE (remote wakeup) */
            ok(io); return;
        }
        break;

    case 0x80:                                  /* standard, device, IN */
        switch (s->bRequest) {
        case 0x00:                              /* GET_STATUS: self powered */
            buf[0] = 1; buf[1] = 0;
            reply_data(io, buf, 2); return;
        case 0x08:                              /* GET_CONFIGURATION */
            buf[0] = drv.rh_config;
            reply_data(io, buf, 1); return;
        case 0x06:                              /* GET_DESCRIPTOR */
            switch (wValue >> 8) {
            case 1: reply_data(io, rh_dev_desc, sizeof(rh_dev_desc)); return;
            case 2: reply_data(io, rh_cfg_desc, sizeof(rh_cfg_desc)); return;
            }
            break;
        }
        break;

    case 0x81:                                  /* standard, interface, IN */
    case 0x82:                                  /* standard, endpoint, IN */
        if (s->bRequest == 0x00) {
            buf[0] = 0; buf[1] = 0;
            reply_data(io, buf, 2); return;
        }
        break;

    case 0x01:                                  /* standard, interface, OUT */
    case 0x02:                                  /* standard, endpoint, OUT */
        ok(io); return;

    case 0x20:                                  /* class, hub, OUT */
        ok(io); return;

    case 0xA0:                                  /* class, hub, IN */
        if (s->bRequest == 0x00) {              /* GET_STATUS */
            buf[0] = buf[1] = buf[2] = buf[3] = 0;
            reply_data(io, buf, 4); return;
        }
        if (s->bRequest == 0x06 && (wValue >> 8) == 0x29) {
            reply_data(io, rh_hub_desc, sizeof(rh_hub_desc)); return;
        }
        break;

    case 0xA3:                                  /* class, port, IN */
        if (s->bRequest == 0x00 && wIndex == 1) {
            UWORD st;
            rh_update_changes();
            st = port_status();
            buf[0] = (UBYTE)st;
            buf[1] = (UBYTE)(st >> 8);
            buf[2] = (UBYTE)drv.rh_change;
            buf[3] = (UBYTE)(drv.rh_change >> 8);
            reply_data(io, buf, 4); return;
        }
        break;

    case 0x23:                                  /* class, port, OUT */
        if (wIndex != 1)
            break;
        if (s->bRequest == 0x03) {              /* SET_FEATURE */
            switch (wValue) {
            case PF_POWER:
                hw_port_power();
                ok(io); return;
            case PF_RESET:
                chan_halt();
                if (hw_port_reset()) {
                    io->iouh_Req.io_Error = UHIOERR_HOSTERROR;
                    return;
                }
                drv.rh_change |= 1 << (PF_C_RESET - 16);
                rh_update_changes();
                ok(io); return;
            case PF_ENABLE:
            case PF_SUSPEND:
                ok(io); return;
            }
        } else if (s->bRequest == 0x01) {       /* CLEAR_FEATURE */
            switch (wValue) {
            case PF_C_CONNECTION:
                drv.rh_change &= ~0x0001;
                hw_hprt_write(HPRT_CONNDET, 0);
                ok(io); return;
            case PF_C_ENABLE:
                drv.rh_change &= ~0x0002;
                hw_hprt_write(HPRT_ENACHG, 0);
                ok(io); return;
            case PF_C_SUSPEND:
                drv.rh_change &= ~0x0004;
                ok(io); return;
            case PF_C_OVERCURRENT:
                drv.rh_change &= ~0x0008;
                hw_hprt_write(HPRT_OVRCHG, 0);
                ok(io); return;
            case PF_C_RESET:
                drv.rh_change &= ~0x0010;
                ok(io); return;
            case PF_POWER:
                hw_hprt_write(0, HPRT_PWR);
                ok(io); return;
            case PF_ENABLE:
                hw_hprt_write(HPRT_ENA, 0);     /* W1C: disables the port */
                ok(io); return;
            case PF_SUSPEND:
                ok(io); return;
            }
        }
        break;
    }
    stall(io);
}
