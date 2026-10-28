/*
 * B3Diag - show berrypi3ap.device statistics
 * Copyright (C) 2026 MichiB210
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Usage:  B3Diag          show counters
 *         B3Diag CLEAR    reset counters (then do your test, then B3Diag)
 */
#include <stdio.h>
#include <string.h>
#include <exec/types.h>
#include <exec/io.h>
#include <exec/errors.h>
#include <proto/exec.h>
#include <devices/usbhardware.h>
#include "../src/b3stats.h"

static const char ver[] __attribute__((used)) = "$VER: B3Diag 1.7 (28.10.2026)";

static unsigned long pct(unsigned long a, unsigned long b)
{
    return b ? (a * 100UL + b / 2) / b : 0;
}

int main(int argc, char **argv)
{
    struct MsgPort *mp;
    struct IOUsbHWReq *io;
    struct B3Stats st;
    int clear = 0;
    static const char *spd[] = { "high speed (480)", "full speed (12)", "low speed (1.5)" };

    if (argc > 1) {
        /* 1.4: accept CLEAR, CLEAN and RESET in any case */
        char a[8];
        int i;
        for (i = 0; i < 7 && argv[1][i]; i++)
            a[i] = (argv[1][i] >= 'a' && argv[1][i] <= 'z') ? argv[1][i] - 32 : argv[1][i];
        a[i] = 0;
        if (!strcmp(a, "CLEAR") || !strcmp(a, "CLEAN") || !strcmp(a, "RESET"))
            clear = 1;
        else {
            printf("Usage: B3Diag [CLEAR]\n");
            return 5;
        }
    }

    mp = CreateMsgPort();
    if (!mp)
        return 20;
    io = (struct IOUsbHWReq *)CreateIORequest(mp, sizeof(struct IOUsbHWReq));
    if (!io) {
        DeleteMsgPort(mp);
        return 20;
    }
    if (OpenDevice((STRPTR)"berrypi3ap.device", 0, (struct IORequest *)io, 0)) {
        printf("B3Diag: cannot open berrypi3ap.device (is Poseidon using it?)\n");
        DeleteIORequest((struct IORequest *)io);
        DeleteMsgPort(mp);
        return 10;
    }

    if (clear) {
        io->iouh_Req.io_Command = B3CMD_CLEARSTATS;
        DoIO((struct IORequest *)io);
        if (io->iouh_Req.io_Error)
            printf("B3Diag: driver does not support statistics (needs 103.3+)\n");
        else
            printf("Counters cleared. Now do the test, then run B3Diag.\n");
        goto done;
    }

    memset(&st, 0, sizeof(st));
    io->iouh_Req.io_Command = B3CMD_GETSTATS;
    io->iouh_Data = &st;
    io->iouh_Length = sizeof(st);
    DoIO((struct IORequest *)io);
    if (io->iouh_Req.io_Error || st.version < 1) {
        printf("B3Diag: driver does not support statistics (needs 103.3+)\n");
        goto done;
    }

    printf("berrypi3ap.device %u.%u   up %lu.%01lu s   port: %s   root hub addr %u\n\n",
           st.drv_version, st.drv_revision,
           (unsigned long)(st.uptime_ms / 1000), (unsigned long)((st.uptime_ms % 1000) / 100),
           st.port_speed < 3 ? spd[st.port_speed] : "?", st.rh_addr);

    printf("Interrupt (mouse/keyboard/hub)\n");
    printf("  polls %lu   reports %lu   nothing new %lu   errors %lu   failed %lu\n",
           (unsigned long)st.intr_polls, (unsigned long)st.intr_data,
           (unsigned long)st.intr_nak, (unsigned long)st.intr_err,
           (unsigned long)st.intr_fail);
    printf("  poll late  <1ms %lu  1-2 %lu  2-4 %lu  4-8 %lu  8-16 %lu  >=16 %lu   (max %lu ms)\n\n",
           (unsigned long)st.late[0], (unsigned long)st.late[1], (unsigned long)st.late[2],
           (unsigned long)st.late[3], (unsigned long)st.late[4], (unsigned long)st.late[5],
           (unsigned long)st.late_max_ms);

    printf("Split (devices behind a USB 2.0 hub)\n");
    printf("  ok %lu   device NAK %lu   hub busy %lu   CSPLIT NYET %lu\n",
           (unsigned long)st.split_ok, (unsigned long)st.split_nak,
           (unsigned long)st.split_tt_busy, (unsigned long)st.split_cs_nyet);
    printf("  NYET give-up %lu (%lu%% of polls)   xact errors %lu   failed %lu\n\n",
           (unsigned long)st.split_nyet_giveup,
           pct(st.split_nyet_giveup, st.split_ok + st.split_nak + st.split_nyet_giveup),
           (unsigned long)st.split_xacterr, (unsigned long)st.split_err);

    printf("Control  requests %lu   errors %lu\n",
           (unsigned long)st.ctrl_reqs, (unsigned long)st.ctrl_err);
    printf("Bulk     slices %lu   %lu KB   errors %lu\n",
           (unsigned long)st.bulk_slices, (unsigned long)(st.bulk_bytes >> 10),
           (unsigned long)st.bulk_err);
    printf("Other    retries %lu   toggle errors %lu   timer wakeups %lu\n",
           (unsigned long)st.xact_retry, (unsigned long)st.toggle_err,
           (unsigned long)st.timer_wakeups);
    if (st.version >= 2)
        printf("v2       toggle re-poll: hub %lu  direct %lu   late complete-split ok %lu\n",
               (unsigned long)st.tgl_in_split, (unsigned long)st.tgl_in_direct,
               (unsigned long)st.split_cs_late_ok);
    if (st.version >= 3)
        printf("v3       completed while halting (rescued) %lu\n",
               (unsigned long)st.halt_race_ok);
    if (st.version >= 4) {
        printf("\nTiming (longest single step, ms)\n");
        printf("  control %lu (>=8ms: %lu)   bulk slice %lu (>=8ms: %lu)   interrupt poll %lu\n",
               (unsigned long)st.ctrl_max_ms, (unsigned long)st.slow_ctrl,
               (unsigned long)st.bulk_max_ms, (unsigned long)st.slow_bulk,
               (unsigned long)st.intr_max_ms);
        printf("  timer late max %lu (>=4ms: %lu)   pauses %lu\n",
               (unsigned long)st.timer_late_max_ms, (unsigned long)st.timer_late_cnt,
               (unsigned long)st.pauses);
    }
    if (st.version >= 5) {
        unsigned long ms = st.uptime_ms - st.clear_ms;
        if (st.clear_ms == 0 || ms > st.uptime_ms)
            ms = st.uptime_ms;
        printf("\nInterrupt wake-up (Pi system timer)\n");
        printf("  %s   interrupts %lu (%lu per second)   SOF %lu   other %lu   storm-off %lu\n",
               st.irq_enabled ? "ON " : "OFF",
               (unsigned long)st.irq_count,
               ms ? (unsigned long)((st.irq_count * 10UL) / (ms / 100UL ? ms / 100UL : 1)) : 0UL,
               (unsigned long)st.irq_sof, (unsigned long)st.irq_other,
               (unsigned long)st.irq_storm);
    }
    if (st.version >= 6)
        printf("  timer irqs %lu   armed late %lu   safety net %lu   missed %lu\n",
               (unsigned long)st.irq_timer, (unsigned long)st.irq_wake_now,
               (unsigned long)st.watchdog, (unsigned long)st.irq_lost);
    if (st.version >= 7) {
        printf("\nPeriodic engine (mouse/keyboard/hub polls in the interrupt)\n");
        printf("  %s   channel irqs %lu   timer irqs %lu   microframe waits %lu\n",
               st.pe_on ? "ON " : "OFF (task polls)",
               (unsigned long)st.pe_chan_irq, (unsigned long)st.pe_timer_irq,
               (unsigned long)st.pe_uf_wait);
        printf("  stuck (freed by watchdog) %lu   spurious %lu\n",
               (unsigned long)st.pe_stuck, (unsigned long)st.pe_spurious);
    }

done:
    CloseDevice((struct IORequest *)io);
    DeleteIORequest((struct IORequest *)io);
    DeleteMsgPort(mp);
    return 0;
}
