/*
 * berrypi3ap.device - statistics shared with the B3Diag tool
 * Copyright (C) 2026 MichiB210
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef B3STATS_H
#define B3STATS_H

#include <exec/types.h>

/* private device commands (outside Poseidon's UHCMD range) */
#define B3CMD_GETSTATS   0x4233     /* copy struct B3Stats to iouh_Data */
#define B3CMD_CLEARSTATS 0x4234     /* zero all counters */

#define B3STATS_VERSION  7

struct B3Stats {
    UWORD  version;             /* B3STATS_VERSION */
    UWORD  drv_version, drv_revision;
    UBYTE  port_speed;          /* 0 high, 1 full, 2 low (root port partner) */
    UBYTE  rh_addr;
    ULONG  uptime_ms;

    /* interrupt endpoints (mouse, keyboard, hubs) */
    ULONG  intr_polls;          /* transactions tried */
    ULONG  intr_data;           /* reports delivered */
    ULONG  intr_nak;            /* nothing new */
    ULONG  intr_err;            /* error polls (tolerated) */
    ULONG  intr_fail;           /* requests failed after retries */
    ULONG  late[6];             /* poll start after due: <1,1-2,2-4,4-8,8-16,>=16 ms */
    ULONG  late_max_ms;

    /* split transactions (devices behind a high-speed hub) */
    ULONG  split_ok;
    ULONG  split_nak;           /* device had nothing */
    ULONG  split_tt_busy;       /* hub refused the start-split */
    ULONG  split_cs_nyet;       /* single complete-split NYETs */
    ULONG  split_nyet_giveup;   /* all complete-splits NYET -> poll later */
    ULONG  split_xacterr;       /* transaction errors, retried */
    ULONG  split_err;           /* gave up after retries */

    /* control / bulk */
    ULONG  ctrl_reqs, ctrl_err;
    ULONG  bulk_slices, bulk_bytes, bulk_err;
    ULONG  xact_retry;
    ULONG  toggle_err;          /* DATATGLERR seen */
    ULONG  timer_wakeups;

    /* version 2 */
    ULONG  tgl_in_split;        /* toggle mismatch on IN behind a hub (re-polled) */
    ULONG  tgl_in_direct;       /* toggle mismatch on IN, direct (re-polled) */
    ULONG  split_cs_late_ok;    /* data arrived on complete-split 4..6 */

    /* version 3 */
    ULONG  halt_race_ok;        /* transfer completed while being halted */

    /* version 4: where does the time go? (all in ms) */
    ULONG  ctrl_max_ms;         /* longest single control request */
    ULONG  bulk_max_ms;         /* longest single bulk slice */
    ULONG  intr_max_ms;         /* longest single interrupt poll */
    ULONG  timer_late_max_ms;   /* timer woke us later than asked */
    ULONG  timer_late_cnt;      /* ... by 4 ms or more */
    ULONG  pauses;              /* 1 ms pauses after 16 ms of work */
    ULONG  slow_ctrl;           /* control requests taking >= 8 ms */
    ULONG  slow_bulk;           /* bulk slices taking >= 8 ms */

    /* version 5: USB interrupt path (stage 3) */
    ULONG  clear_ms;            /* uptime_ms at the last CLEAR */
    ULONG  irq_enabled;         /* 1 while the USB interrupt is on */
    ULONG  irq_count;           /* USB interrupts handled */
    ULONG  irq_sof;             /* ... start-of-frame */
    ULONG  irq_other;           /* ... unexpected causes (masked) */
    ULONG  irq_storm;           /* storm guard switched the interrupt off */

    /* version 6: wake-up by the Pi system timer */
    ULONG  irq_timer;           /* system timer interrupts */
    ULONG  irq_wake_now;        /* target already passed when armed */
    ULONG  watchdog;            /* timer.device safety net fired */
    ULONG  irq_lost;            /* ... and a system timer wake was overdue */

    /* version 7: periodic engine (interrupt polls run in the interrupt) */
    ULONG  pe_on;               /* 1 while the engine runs the polls */
    ULONG  pe_chan_irq;         /* channel 1 halt interrupts */
    ULONG  pe_timer_irq;        /* engine timer (system timer C1) */
    ULONG  pe_uf_wait;          /* waits for microframe 7/0 before a split */
    ULONG  pe_stuck;            /* watchdog freed a stuck poll */
    ULONG  pe_spurious;         /* channel 1 interrupt without a poll */
};

#endif
