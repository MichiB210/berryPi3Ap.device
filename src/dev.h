/*
 * berrypi3ap.device - USB host controller driver for PiStorm/Emu68 (Pi 3A+)
 * Copyright (C) 2026 MichiB210
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef DEV_H
#define DEV_H

#include <exec/types.h>
#include <dos/dos.h>
#include <exec/devices.h>
#include <exec/io.h>
#include <exec/lists.h>
#include <exec/ports.h>
#include <exec/tasks.h>
#include <devices/timer.h>
#include <devices/usbhardware.h>
#include "b3stats.h"

#define DEVNAME         "berrypi3ap.device"
#define DEV_VERSION     105
#define DEV_REVISION    1
#define DEV_DATE        "28.10.2026"

/* Not in every usbhardware.h: capability tag used since USB57 */
#ifndef UHA_Capabilities
#define UHA_Capabilities (UHA_Dummy + 0x21)
#endif
#define UHCF_USB20      0x00000001UL

#define MIN_IOREQ_SIZE  90      /* V2 IOUsbHWReq */

/* Sizes of the controller-owned DMA areas */
#define DATA_AREA_SIZE  (64UL * 1024UL + 1024UL)   /* ctrl + bulk chunks */
#define INTR_AREA_SIZE  1024UL

#ifndef ROMBUILD
struct DevBase {
    struct Device db_Dev;
    BPTR          db_SegList;
};
#endif

/* Global driver state - single unit, single instance. */
struct Drv {
    struct DevBase     *base;
    struct Task        *task;           /* unit task */
    struct MsgPort     *port;           /* BeginIO -> unit task */
    struct MsgPort     *tport;          /* timer replies */
    struct timerequest *treq;
    BOOL                timer_busy;

    struct List         ctrl_q;         /* control requests to real devices */
    struct List         bulk_q;
    struct List         intr_q;         /* periodic polls, non-blocking */
    struct List         rh_intr_q;      /* root hub status change requests */

    struct IOUsbHWReq  *current;        /* request executing right now */
    volatile BOOL       abort_current;  /* AbortIO hit the running one */

    ULONG               reset_sigmask;  /* keyboard reset handler -> task */
    ULONG               wake_sigmask;   /* system timer interrupt -> task */

    struct Task        *opener;         /* waits for init result */
    BYTE                opener_sig;
    LONG                init_result;
    BOOL                hw_ok;

    UWORD               state;          /* UHSF_* */

    /* virtual root hub */
    UBYTE               rh_addr;
    UBYTE               rh_config;
    UWORD               rh_change;      /* wPortChange bits */
    UBYTE               rh_last_conn;
    UBYTE               rh_reset_done;

    /* root port partner speed: 0 high, 1 full, 2 low (HPRT encoding) */
    UBYTE               port_speed;

    /* DMA areas (64-byte aligned, from fast RAM) */
    APTR                dma_alloc;
    ULONG               dma_alloc_size;
    UBYTE              *setup_buf;      /* 64 bytes */
    UBYTE              *status_buf;     /* 64 bytes */
    UBYTE              *intr_buf;       /* INTR_AREA_SIZE */
    UBYTE              *data_buf;       /* DATA_AREA_SIZE */

    APTR                prop_alloc;
    ULONG              *prop;

    /* DATA0/1 toggles: bit (addr*16+ep), IN and OUT separately */
    UBYTE               tog_in[256];
    UBYTE               tog_out[256];

    /* statistics, read by B3Diag (see b3stats.h) */
    struct B3Stats      st;
};

#ifndef ROMBUILD
extern struct Drv drv;
extern struct ExecBase *SysBase;
#endif

static inline void new_list(struct List *l)
{
    l->lh_Head = (struct Node *)&l->lh_Tail;
    l->lh_Tail = NULL;
    l->lh_TailPred = (struct Node *)&l->lh_Head;
}

/* helpers for the IOUsbHWReq private fields
 * REQ_DUE:   bits 0-13  frame number of the next allowed poll/slice
 *            bits 16-23 consecutive transaction error count
 *            bit 24     due time valid
 *            bit 25     bulk OUT: use PING next time (high speed, after NYET)
 *            bit 26     bulk: data was flowing on the last slice
 * REQ_START: millisecond time stamp + 1 of the last progress (NAK timeout)
 */
#define REQ_DUE(r)      (*(ULONG *)&(r)->iouh_DriverPrivate1)
#define REQ_START(r)    (*(ULONG *)&(r)->iouh_DriverPrivate2)
#define DUE_FRAME_MASK  0x00003FFFUL
#define DUE_ERR_MASK    0x00FF0000UL
#define DUE_VALID       0x01000000UL
#define DUE_PING        0x02000000UL
#define DUE_ACTIVE      0x04000000UL
#define DUE_KEEP_MASK   (DUE_ERR_MASK | DUE_PING | DUE_ACTIVE)

/* hw.c */
ULONG  hw_rd(ULONG off);
void   hw_wr(ULONG off, ULONG val);
LONG   hw_init(void);
void   hw_port_power(void);
LONG   hw_port_reset(void);
ULONG  hw_hprt(void);
void   hw_hprt_write(ULONG set_bits, ULONG clear_bits);
UWORD  hw_frnum(void);
ULONG  hw_ms_now(void);
ULONG  hw_ticks_per_ms(void);
UWORD  hw_frame_diff(UWORD later, UWORD earlier);
void   hw_wait_next_tick(void);
APTR   hw_dma_pre(APTR addr, ULONG len, BOOL to_device, ULONG *bus);
void   hw_dma_post(APTR addr, ULONG len, BOOL to_device);
void   hw_delay_ms(ULONG ms);

/* channel.c */
struct chan_req {
    UBYTE  addr, ep, eptype;
    BOOL   in;
    BOOL   lowspeed;
    UWORD  mps;
    ULONG  hctsiz;          /* size | pktcnt | pid */
    ULONG  bus;             /* DMA bus address */
    BOOL   split;
    BOOL   complete;        /* CSPLIT */
    UBYTE  hub, port;
    BOOL   periodic_next;   /* schedule periodic for the next (micro)frame */
};
void   chan_start(const struct chan_req *c);
ULONG  chan_wait(ULONG max_ticks);
ULONG  chan_halt(void);
ULONG  chan_hctsiz(void);
void   chan1_start(const struct chan_req *c);
void   chan1_disable(void);
BOOL   chan1_busy(void);
ULONG  chan1_hctsiz(void);

/* periodic engine state (xfer.c) */
struct PeState {
    struct IOUsbHWReq *cur;
    UBYTE  phase, cs, errs;
    BOOL   abort, in, split;
    UWORD  ss_frame;
    ULONG  size_ss, len, mps, start_us;
    struct chan_req c;
};

#ifdef ROMBUILD
/* ROM build: a Kickstart module has no writable data or BSS, so every
 * variable lives in the device base (allocated by MakeLibrary from the
 * init table) and the code reaches it through a4.  Every way into the
 * driver loads a4 and restores the caller's value (start.S): the device
 * vectors (a6 = base), the interrupt servers (is_Data = base), the unit
 * task (tc_UserData = base) and the ColdReboot patch (FindName). */
#include <exec/interrupts.h>

struct RomVars {
    /* device.c */
    struct timerequest *tsleep;
    struct MsgPort     *sport;
    ULONG               timer_armed_ms, timer_asked_ms;
    ULONG               wake_target;
    BOOL                wake_armed, watchdog_running;
    BOOL                starting;
    /* hw.c, hw_ms_now() */
    UWORD               last;
    ULONG               ticks_acc, ms;
    /* irq.c */
    struct Interrupt    irq_server;
    struct Interrupt    kbd_handler;
    struct IOStdReq    *kbd_req;
    BOOL                server_added, kbd_added;
    APTR                irq_old_coldreboot;
    volatile BOOL       irq_on;
    UWORD               storm_frame, storm_cnt;
    /* xfer.c */
    volatile BOOL       pe_enabled;
    struct PeState      pe;
};

struct DevBase {
    struct Device       db_Dev;
    BPTR                db_SegList;
    struct ExecBase    *db_SysBase;
    struct Drv          db_Drv;
    struct RomVars      db_Vars;
};

register struct DevBase *rom_base __asm__("a4");

#define SysBase     (rom_base->db_SysBase)
#define drv         (rom_base->db_Drv)
#define RV(x)       (rom_base->db_Vars.x)
#define pe_enabled  RV(pe_enabled)
#define ROM_DATA    ((APTR)rom_base)
#else
#define ROM_DATA    NULL
#endif

/* toggles.c (in xfer.c) */
BOOL   tog_get(UBYTE addr, UBYTE ep, BOOL in);
void   tog_set(UBYTE addr, UBYTE ep, BOOL in, BOOL data1);
void   tog_clear_dev(UBYTE addr);

/* xfer.c */
void   xfer_control(struct IOUsbHWReq *io);
/* returns TRUE when the request is finished (replied by caller) */
BOOL   xfer_bulk_step(struct IOUsbHWReq *io);
BOOL   xfer_intr_poll(struct IOUsbHWReq *io);
BOOL   xfer_intr_due(struct IOUsbHWReq *io, UWORD now);

/* periodic engine (xfer.c, 105.0): interrupt polls on channel 1, run
 * from the interrupt.  pe_kick/pe_on_* need interrupts disabled. */
#ifndef ROMBUILD
extern volatile BOOL pe_enabled;
#endif
void   pe_kick(void);
void   pe_on_timer(void);
void   pe_on_chan(ULONG hcint);
BOOL   pe_abort(struct IOUsbHWReq *io);  /* TRUE if it was the engine's */
void   pe_watchdog(void);
void   pe_reset(void);
BOOL   pe_is_current(struct IOUsbHWReq *io);

/* rh.c */
void   rh_control(struct IOUsbHWReq *io);
void   rh_update_changes(void);
BOOL   rh_intr_try(struct IOUsbHWReq *io);   /* TRUE = completed */
BOOL   rh_is_addr(UWORD addr);

/* irq.c */
void   irq_init(void);
void   irq_off(void);
void   irq_pi_off(void);
void   irq_reset_done(void);
BOOL   irq_usable(void);
ULONG  irq_wake_in(ULONG us);
ULONG  irq_us_now(void);
void   irq_c1_at(ULONG us);         /* periodic engine timer */

/* device.c */
void   req_reply(struct IOUsbHWReq *io);
UWORD  map_hcint_error(ULONG hcint);

#endif
