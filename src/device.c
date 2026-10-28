/*
 * berrypi3ap.device - USB host controller driver for PiStorm/Emu68 (Pi 3A+)
 * Copyright (C) 2026 MichiB210
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Exec device frame and unit task.
 *
 * BeginIO only queues transfers and returns at once.  A dedicated unit
 * task (priority 15) owns DWC2 channel 0 and works the queues:
 *   - control requests are run one at a time,
 *   - bulk requests run in short time slices (round robin, back-off when a
 *     device only NAKs, e.g. an idle network adapter),
 *   - root hub status requests complete when HPRT0 reports a change.
 * Interrupt endpoints (HID, hubs) are polled by the periodic engine on
 * channel 1 inside the interrupt (xfer.c, since 105.0).  The task is
 * woken by the Pi system timer (irq.c, since 104.4); timer.device is
 * only a safety net.
 */
#include <exec/types.h>
#include <exec/resident.h>
#include <exec/memory.h>
#include <exec/execbase.h>
#include <exec/errors.h>
#include <utility/tagitem.h>
#include <dos/dos.h>
#include <proto/exec.h>
#include "dev.h"
#include "regs.h"

struct ExecBase *SysBase;
struct Drv drv;

#define STR2(x) #x
#define STR(x) STR2(x)

static const char dev_name[] = DEVNAME;
static const char dev_idstring[] =
    "berrypi3ap.device " STR(DEV_VERSION) "." STR(DEV_REVISION)
    " (" DEV_DATE ") MichiB210 - C rebuild stage 1 (queued, unit task)\r\n";
static const char dev_vertag[] __attribute__((used)) =
    "$VER: berrypi3ap.device " STR(DEV_VERSION) "." STR(DEV_REVISION)
    " (" DEV_DATE ") GPL-2.0-or-later";

static const char str_vendor[]  = "Broadcom";
static const char str_product[] = "BCM2837 DWC2";
static const char str_desc[]    = "Raspberry Pi 3 DWC2 USB 2.0 Host Controller Driver";
static const char str_copy[]    = "GPL-2.0-or-later (C) 2026 MichiB210";

/* assembler entry stubs (start.S) */
extern void dev_init_stub(void);
extern void dev_open_stub(void);
extern void dev_close_stub(void);
extern void dev_expunge_stub(void);
extern void dev_null_stub(void);
extern void dev_beginio_stub(void);
extern void dev_abortio_stub(void);

static const APTR dev_functable[] = {
    (APTR)dev_open_stub,
    (APTR)dev_close_stub,
    (APTR)dev_expunge_stub,
    (APTR)dev_null_stub,
    (APTR)dev_beginio_stub,
    (APTR)dev_abortio_stub,
    (APTR)-1
};

static const ULONG dev_inittable[4] = {
    sizeof(struct DevBase),
    (ULONG)dev_functable,
    0,
    (ULONG)dev_init_stub
};

const struct Resident dev_romtag __attribute__((used)) = {
    RTC_MATCHWORD,
    (struct Resident *)&dev_romtag,
    (APTR)(&dev_romtag + 1),
    RTF_AUTOINIT,
    DEV_VERSION,
    NT_DEVICE,
    0,
    (char *)dev_name,
    (char *)dev_idstring,
    (APTR)dev_inittable
};

/* ------------------------------------------------------------ helpers */

void req_reply(struct IOUsbHWReq *io)
{
    ReplyMsg(&io->iouh_Req.io_Message);
}

/* 105.0: the queues are also used from the interrupt (periodic engine),
 * so they are protected with Disable() instead of Forbid(). */
static void list_add(struct List *l, struct IOUsbHWReq *io)
{
    Disable();
    AddTail(l, &io->iouh_Req.io_Message.mn_Node);
    if (l == &drv.intr_q)
        pe_kick();                      /* no-op if the engine is off/busy */
    Enable();
}

void hw_delay_ms(ULONG ms);     /* below, needs the unit task's timer */

static struct timerequest *tsleep;
static struct MsgPort *sport;

void hw_delay_ms(ULONG ms)
{
    if (!tsleep) {                      /* no timer yet: crude spin */
        ULONG n;
        for (n = 0; n < ms * 20000UL; n++)
            (void)hw_rd(GSNPSID);
        return;
    }
    tsleep->tr_node.io_Command = TR_ADDREQUEST;
    tsleep->tr_time.tv_secs = ms / 1000;
    tsleep->tr_time.tv_micro = (ms % 1000) * 1000;
    DoIO((struct IORequest *)tsleep);
}

/* ---------------------------------------------------------- dispatch */

static void do_usbreset(struct IOUsbHWReq *io)
{
    drv.state = UHSF_RESET;
    chan_halt();
    pe_reset();
    hw_port_power();
    drv.rh_addr = 0;
    drv.rh_config = 0;
    drv.rh_change = 0;
    {
        ULONG i;
        for (i = 0; i < 256; i++)
            drv.tog_in[i] = drv.tog_out[i] = 0;
    }
    rh_update_changes();
    drv.state = UHSF_OPERATIONAL;
    io->iouh_State = UHSF_OPERATIONAL;
    io->iouh_Flags = (UWORD)((io->iouh_Flags & ~3) | UHFF_HIGHSPEED);
    io->iouh_Actual = 0;
    io->iouh_Req.io_Error = 0;
}

static void dispatch(struct IOUsbHWReq *io)
{
    REQ_DUE(io) = 0;
    REQ_START(io) = 0;

    switch (io->iouh_Req.io_Command) {
    case UHCMD_USBRESET:
    case CMD_RESET:
        do_usbreset(io);
        req_reply(io);
        return;

    case UHCMD_CONTROLXFER:
        if (rh_is_addr(io->iouh_DevAddr)) {
            rh_control(io);
            req_reply(io);
        } else
            list_add(&drv.ctrl_q, io);
        return;

    case UHCMD_INTXFER:
        if (rh_is_addr(io->iouh_DevAddr)) {
            if (io->iouh_Endpoint != 1 || !io->iouh_Data || !io->iouh_Length) {
                io->iouh_Req.io_Error = UHIOERR_BADPARAMS;
                req_reply(io);
            } else if (rh_intr_try(io))
                req_reply(io);
            else
                list_add(&drv.rh_intr_q, io);
        } else
            list_add(&drv.intr_q, io);
        return;

    case UHCMD_BULKXFER:
        list_add(&drv.bulk_q, io);
        return;

    default:                            /* UHCMD_ISOXFER: later stage */
        io->iouh_Req.io_Error = IOERR_NOCMD;
        req_reply(io);
        return;
    }
}

/* Finish the request that was just worked on; honours AbortIO.
 * Returns TRUE if the request is still queued afterwards. */
static BOOL finish_current(struct IOUsbHWReq *io, BOOL done)
{
    BOOL aborted;
    Disable();
    aborted = drv.abort_current;
    drv.abort_current = FALSE;
    drv.current = NULL;
    if (aborted) {
        io->iouh_Req.io_Error = IOERR_ABORTED;
        done = TRUE;
    }
    if (done)
        Remove(&io->iouh_Req.io_Message.mn_Node);
    Enable();
    if (done)
        req_reply(io);
    return !done;
}

/* Pick the first request of a list that is due, and make it current, in
 * one Forbid section so AbortIO can never pull it away underneath us. */
static struct IOUsbHWReq *pick(struct List *l, UWORD now, BOOL check_due)
{
    struct Node *n;
    struct IOUsbHWReq *io = NULL;
    Disable();
    for (n = l->lh_Head; n->ln_Succ; n = n->ln_Succ) {
        struct IOUsbHWReq *r = (struct IOUsbHWReq *)n;
        if (!check_due || xfer_intr_due(r, now)) {
            io = r;
            break;
        }
    }
    if (io) {
        drv.current = io;
        drv.abort_current = FALSE;
    }
    Enable();
    return io;
}

/* ------------------------------------------------------------ scheduler */

/* Returns TRUE if there is work that should run without sleeping. */
static BOOL do_work(void)
{
    struct IOUsbHWReq *io;
    BOOL busy = FALSE;
    ULONG guard;

    drv.st.uptime_ms = hw_ms_now();     /* keep the ms clock running */

    /* root hub status changes: complete every waiting status request */
    rh_update_changes();
    if (drv.rh_change) {
        struct List done;
        struct Node *n;
        new_list(&done);
        Forbid();
        while ((n = RemHead(&drv.rh_intr_q))) {
            rh_intr_try((struct IOUsbHWReq *)n);
            AddTail(&done, n);
        }
        Permit();
        while ((n = RemHead(&done)))
            req_reply((struct IOUsbHWReq *)n);
    }

    /* 105.0: interrupt endpoints are polled by the periodic engine in
     * the interrupt.  The task only watches it, and polls itself as a
     * fallback when the interrupt is off. */
    pe_watchdog();
    for (guard = 0; !pe_enabled && guard < 32; guard++) {
        UWORD now = hw_frnum();
        BOOL done;
        io = pick(&drv.intr_q, now, TRUE);
        if (!io)
            break;
        if (REQ_DUE(io) & DUE_VALID) {
            ULONG late = hw_frame_diff(now, (UWORD)(REQ_DUE(io) & DUE_FRAME_MASK))
                         / hw_ticks_per_ms();
            int b = late < 1 ? 0 : late < 2 ? 1 : late < 4 ? 2 :
                    late < 8 ? 3 : late < 16 ? 4 : 5;
            drv.st.late[b]++;
            if (late > drv.st.late_max_ms)
                drv.st.late_max_ms = late;
        }
        {
            ULONG t0 = hw_ms_now(), dt;
            done = xfer_intr_poll(io);
            dt = hw_ms_now() - t0;
            if (dt > drv.st.intr_max_ms)
                drv.st.intr_max_ms = dt;
        }
        if (done && io->iouh_Req.io_Error)
            drv.st.intr_fail++;
        finish_current(io, done);
    }

    /* one control request */
    io = pick(&drv.ctrl_q, 0, FALSE);
    if (io) {
        ULONG t0 = hw_ms_now(), dt;
        xfer_control(io);
        dt = hw_ms_now() - t0;
        if (dt > drv.st.ctrl_max_ms)
            drv.st.ctrl_max_ms = dt;
        if (dt >= 8)
            drv.st.slow_ctrl++;
        drv.st.ctrl_reqs++;
        if (io->iouh_Req.io_Error)
            drv.st.ctrl_err++;
        finish_current(io, TRUE);
        busy = TRUE;
    }

    /* one bulk slice; requests that only NAK back off for ~1 ms */
    io = pick(&drv.bulk_q, hw_frnum(), TRUE);
    if (io) {
        ULONG before = io->iouh_Actual;
        ULONG t0 = hw_ms_now(), dt;
        BOOL done = xfer_bulk_step(io);
        dt = hw_ms_now() - t0;
        if (dt > drv.st.bulk_max_ms)
            drv.st.bulk_max_ms = dt;
        if (dt >= 8)
            drv.st.slow_bulk++;
        if (done && io->iouh_Req.io_Error)
            drv.st.bulk_err++;
        if (!done) {
            if (io->iouh_Actual == before)
                REQ_DUE(io) = (REQ_DUE(io) & DUE_KEEP_MASK) | DUE_VALID |
                              ((hw_frnum() + hw_ticks_per_ms()) & DUE_FRAME_MASK);
            else {
                REQ_DUE(io) &= DUE_KEEP_MASK;
                busy = TRUE;
            }
        } else
            busy = TRUE;
        if (finish_current(io, done)) {
            /* still queued: rotate so a second bulk device gets a turn */
            Forbid();
            if (io->iouh_Req.io_Message.mn_Node.ln_Succ->ln_Succ) {
                Remove(&io->iouh_Req.io_Message.mn_Node);
                AddTail(&drv.bulk_q, &io->iouh_Req.io_Message.mn_Node);
            }
            Permit();
        }
    }

    if (!IsListEmpty(&drv.ctrl_q))
        busy = TRUE;
    return busy;
}

static ULONG timer_armed_ms, timer_asked_ms;
static ULONG wake_target;
static BOOL  wake_armed, watchdog_running;

static void note_wake_late(void)
{
    LONG late_us = (LONG)(irq_us_now() - wake_target);
    if (late_us > 0) {
        ULONG over = (ULONG)late_us / 1000;
        if (over > drv.st.timer_late_max_ms)
            drv.st.timer_late_max_ms = over;
        if (over >= 4)
            drv.st.timer_late_cnt++;
    }
}

static void arm_timer(void)
{
    ULONG us;
    if (drv.timer_busy && !watchdog_running)
        return;
    if (!IsListEmpty(&drv.bulk_q))
        us = 1000;
    else if (!pe_enabled && !IsListEmpty(&drv.intr_q)) {
        /* 104.2: sleep exactly until the next interrupt poll is due
         * instead of a fixed 1 ms (fewer useless wake-ups, and we wake
         * at the right moment). */
        struct Node *n;
        UWORD now = hw_frnum();
        ULONG tpm = hw_ticks_per_ms();
        ULONG min_ticks = 0xFFFF;
        Forbid();
        for (n = drv.intr_q.lh_Head; n->ln_Succ; n = n->ln_Succ) {
            struct IOUsbHWReq *r = (struct IOUsbHWReq *)n;
            ULONG t;
            if (xfer_intr_due(r, now))
                t = 0;
            else
                t = hw_frame_diff((UWORD)(REQ_DUE(r) & DUE_FRAME_MASK), now);
            if (t < min_ticks)
                min_ticks = t;
        }
        Permit();
        us = min_ticks * (1000UL / tpm);
        if (us < 125)
            us = 125;
        if (us > 10000)
            us = 10000;
    } else if (!IsListEmpty(&drv.rh_intr_q))
        us = 10000;
    else
        return;

    if (irq_usable()) {
        /* 104.4: one system timer interrupt at the exact moment;
         * timer.device only as a 50 ms safety net */
        SetSignal(0, drv.wake_sigmask);
        wake_target = irq_wake_in(us);
        wake_armed = TRUE;
        if (!drv.timer_busy) {
            drv.treq->tr_node.io_Command = TR_ADDREQUEST;
            drv.treq->tr_time.tv_secs = 0;
            drv.treq->tr_time.tv_micro = 50000;
            SendIO((struct IORequest *)drv.treq);
            drv.timer_busy = TRUE;
            watchdog_running = TRUE;
        }
        return;
    }

    if (drv.timer_busy) {               /* fallback: drop the safety net */
        AbortIO((struct IORequest *)drv.treq);
        WaitIO((struct IORequest *)drv.treq);
        drv.timer_busy = FALSE;
        watchdog_running = FALSE;
    }
    drv.treq->tr_node.io_Command = TR_ADDREQUEST;
    drv.treq->tr_time.tv_secs = 0;
    drv.treq->tr_time.tv_micro = us;
    timer_armed_ms = hw_ms_now();
    timer_asked_ms = (us + 999) / 1000;
    SendIO((struct IORequest *)drv.treq);
    drv.timer_busy = TRUE;
}

static void unit_task(void)
{
    ULONG portsig, tsig;

    drv.port = CreateMsgPort();
    drv.tport = CreateMsgPort();
    sport = CreateMsgPort();
    if (!drv.port || !drv.tport || !sport)
        goto fail;
    drv.treq = (struct timerequest *)CreateIORequest(drv.tport, sizeof(struct timerequest));
    tsleep = (struct timerequest *)CreateIORequest(sport, sizeof(struct timerequest));
    if (!drv.treq || !tsleep)
        goto fail;
    if (OpenDevice((STRPTR)TIMERNAME, UNIT_MICROHZ, (struct IORequest *)drv.treq, 0))
        goto fail;
    tsleep->tr_node.io_Device = drv.treq->tr_node.io_Device;
    tsleep->tr_node.io_Unit = drv.treq->tr_node.io_Unit;

    drv.init_result = hw_init();
    if (drv.init_result)
        goto fail_hw;
    drv.hw_ok = TRUE;
    {
        BYTE rs = AllocSignal(-1);
        drv.reset_sigmask = rs >= 0 ? (1UL << rs) : 0;
    }
    {
        BYTE ws = AllocSignal(-1);
        drv.wake_sigmask = ws >= 0 ? (1UL << ws) : 0;
    }
    irq_init();                         /* 104.4 wake-ups, 105.0 periodic engine */
    drv.state = UHSF_OPERATIONAL;
    rh_update_changes();
    Signal(drv.opener, 1UL << drv.opener_sig);

    portsig = 1UL << drv.port->mp_SigBit;
    tsig = 1UL << drv.tport->mp_SigBit;

    for (;;) {
        struct IOUsbHWReq *io;
        BOOL busy;
        ULONG busy_since;

        {
            ULONG got = Wait(portsig | tsig | drv.reset_sigmask | drv.wake_sigmask);
            if (got & drv.reset_sigmask)
                irq_reset_done();       /* Ctrl-Amiga-Amiga: let the reboot go on */
            if ((got & drv.wake_sigmask) && wake_armed) {
                wake_armed = FALSE;
                drv.st.timer_wakeups++;
                note_wake_late();
            }
        }
        busy_since = hw_ms_now();

        do {
            /* 103.1: clear the port signal BEFORE draining the port.
             * 103.0 never cleared it, so one message arriving while we
             * worked made the loop spin forever and starved every task
             * below priority 5 (only the mouse pointer still moved). */
            SetSignal(0, portsig);
            while ((io = (struct IOUsbHWReq *)GetMsg(drv.port)))
                dispatch(io);
            if (drv.timer_busy && CheckIO((struct IORequest *)drv.treq)) {
                WaitIO((struct IORequest *)drv.treq);
                drv.timer_busy = FALSE;
                if (watchdog_running) {
                    watchdog_running = FALSE;
                    drv.st.watchdog++;
                    if (wake_armed && (LONG)(irq_us_now() - wake_target) > 20000) {
                        drv.st.irq_lost++;      /* system timer wake missed */
                        wake_armed = FALSE;
                    }
                } else {
                drv.st.timer_wakeups++;
                {
                    ULONG slept = hw_ms_now() - timer_armed_ms;
                    if (slept > timer_asked_ms) {
                        ULONG over = slept - timer_asked_ms;
                        if (over > drv.st.timer_late_max_ms)
                            drv.st.timer_late_max_ms = over;
                        if (over >= 4)
                            drv.st.timer_late_cnt++;
                    }
                }
                }
            }
            busy = do_work();

            /* Long continuous work (e.g. a big copy): give lower-priority
             * tasks a moment, so Workbench stays usable. */
            if (busy && hw_ms_now() - busy_since >= 16) {
                hw_delay_ms(1);
                drv.st.pauses++;
                busy_since = hw_ms_now();
            }
        } while (busy || (SetSignal(0, 0) & portsig));

        arm_timer();
    }

fail_hw:
    CloseDevice((struct IORequest *)drv.treq);
fail:
    if (drv.init_result == 0)
        drv.init_result = -10;
    tsleep = NULL;
    Signal(drv.opener, 1UL << drv.opener_sig);
    /* task ends; exec frees it (memory is leaked on this error path) */
}

static LONG start_unit(void)
{
    struct Task *t;
    APTR stack;
    const ULONG stacksize = 16384;

    t = AllocMem(sizeof(struct Task), MEMF_PUBLIC | MEMF_CLEAR);
    stack = AllocMem(stacksize, MEMF_PUBLIC | MEMF_CLEAR);
    if (!t || !stack)
        return -1;

    /* controller-owned DMA memory in Emu68 fast RAM, 64-byte aligned */
    drv.dma_alloc_size = 64 + 64 + 1024 + INTR_AREA_SIZE + DATA_AREA_SIZE + 64;
    drv.dma_alloc = AllocMem(drv.dma_alloc_size, MEMF_FAST | MEMF_PUBLIC | MEMF_CLEAR);
    drv.prop_alloc = AllocMem(512, MEMF_FAST | MEMF_PUBLIC | MEMF_CLEAR);
    if (!drv.dma_alloc || !drv.prop_alloc)
        return -1;
    {
        ULONG a = ((ULONG)drv.dma_alloc + 63) & ~63UL;
        drv.setup_buf  = (UBYTE *)a;            a += 64;
        drv.status_buf = (UBYTE *)a;            a += 1024;
        drv.intr_buf   = (UBYTE *)a;            a += INTR_AREA_SIZE;
        drv.data_buf   = (UBYTE *)a;
        drv.prop = (ULONG *)(((ULONG)drv.prop_alloc + 127) & ~127UL);
    }

    t->tc_Node.ln_Type = NT_TASK;
    t->tc_Node.ln_Pri = 15;              /* 104.1: 15 - timer wake-ups were up to 325 ms late at 5 */
    t->tc_Node.ln_Name = (char *)dev_name;
    t->tc_SPLower = stack;
    t->tc_SPUpper = (APTR)((ULONG)stack + stacksize);
    t->tc_SPReg = t->tc_SPUpper;
    new_list(&t->tc_MemEntry);

    drv.opener = FindTask(NULL);
    drv.opener_sig = AllocSignal(-1);
    if (drv.opener_sig < 0)
        return -1;
    SetSignal(0, 1UL << drv.opener_sig);
    drv.init_result = 0;

    drv.task = t;
    AddTask(t, (APTR)unit_task, NULL);
    Wait(1UL << drv.opener_sig);
    FreeSignal(drv.opener_sig);

    return drv.hw_ok ? 0 : (drv.init_result ? drv.init_result : -1);
}

/* ------------------------------------------------------ device vectors */

struct DevBase *dev_init(struct DevBase *base, BPTR seglist, struct ExecBase *sys)
{
    SysBase = sys;
    base->db_SegList = seglist;
    base->db_Dev.dd_Library.lib_Node.ln_Type = NT_DEVICE;
    base->db_Dev.dd_Library.lib_Node.ln_Name = (char *)dev_name;
    base->db_Dev.dd_Library.lib_Flags = LIBF_SUMUSED | LIBF_CHANGED;
    base->db_Dev.dd_Library.lib_Version = DEV_VERSION;
    base->db_Dev.dd_Library.lib_Revision = DEV_REVISION;
    base->db_Dev.dd_Library.lib_IdString = (APTR)dev_idstring;

    drv.base = base;
    new_list(&drv.ctrl_q);
    new_list(&drv.bulk_q);
    new_list(&drv.intr_q);
    new_list(&drv.rh_intr_q);
    drv.state = UHSF_RESET;
    return base;
}

static BOOL starting;

void dev_open(struct DevBase *base, struct IOUsbHWReq *io, ULONG unit, ULONG flags)
{
    (void)flags;
    io->iouh_Req.io_Error = 0;

    if (unit != 0) {
        io->iouh_Req.io_Error = IOERR_OPENFAIL;
        return;
    }
    if (io->iouh_Req.io_Message.mn_Length < MIN_IOREQ_SIZE) {
        io->iouh_Req.io_Error = IOERR_BADLENGTH;
        return;
    }

    base->db_Dev.dd_Library.lib_OpenCnt++;     /* keep us during init */
    if (!drv.hw_ok) {
        if (starting || drv.task) {
            base->db_Dev.dd_Library.lib_OpenCnt--;
            io->iouh_Req.io_Error = IOERR_OPENFAIL;
            return;
        }
        starting = TRUE;
        if (start_unit() != 0) {
            starting = FALSE;
            base->db_Dev.dd_Library.lib_OpenCnt--;
            io->iouh_Req.io_Error = IOERR_OPENFAIL;
            return;
        }
        starting = FALSE;
    }

    io->iouh_Req.io_Device = &base->db_Dev;
    io->iouh_Req.io_Unit = NULL;
    io->iouh_Req.io_Message.mn_Node.ln_Type = NT_REPLYMSG;
    base->db_Dev.dd_Library.lib_Flags &= ~LIBF_DELEXP;
}

BPTR dev_close(struct DevBase *base, struct IOUsbHWReq *io)
{
    io->iouh_Req.io_Device = (struct Device *)-1;
    io->iouh_Req.io_Unit = (struct Unit *)-1;
    if (base->db_Dev.dd_Library.lib_OpenCnt)
        base->db_Dev.dd_Library.lib_OpenCnt--;
    return 0;                           /* stays resident (owns the hardware) */
}

BPTR dev_expunge(struct DevBase *base)
{
    base->db_Dev.dd_Library.lib_Flags |= LIBF_DELEXP;
    return 0;
}

static ULONG query_device(struct IOUsbHWReq *io)
{
    struct TagItem *ti = (struct TagItem *)io->iouh_Data;
    ULONG count = 0;

    while (ti) {
        ULONG tag = ti->ti_Tag;
        ULONG *p = (ULONG *)ti->ti_Data;
        if (tag == TAG_DONE)
            break;
        if (tag == TAG_MORE) {
            ti = (struct TagItem *)ti->ti_Data;
            continue;
        }
        if (tag == TAG_SKIP) {
            ti += 1 + ti->ti_Data;
            continue;
        }
        if (p) {
            switch (tag) {
            case UHA_State:        *p = drv.state; count++; break;
            case UHA_Manufacturer: *p = (ULONG)str_vendor; count++; break;
            case UHA_ProductName:  *p = (ULONG)str_product; count++; break;
            case UHA_Description:  *p = (ULONG)str_desc; count++; break;
            case UHA_Copyright:    *p = (ULONG)str_copy; count++; break;
            case UHA_Version:      *p = DEV_VERSION; count++; break;
            case UHA_Revision:     *p = DEV_REVISION; count++; break;
            case UHA_DriverVersion:*p = 0x200; count++; break;
            case UHA_Capabilities: *p = UHCF_USB20; count++; break;
            }
        }
        ti++;
    }
    return count;
}

void dev_beginio(struct DevBase *base, struct IOUsbHWReq *io)
{
    (void)base;
    io->iouh_Req.io_Message.mn_Node.ln_Type = NT_MESSAGE;
    io->iouh_Req.io_Error = 0;

    switch (io->iouh_Req.io_Command) {
    case UHCMD_QUERYDEVICE:
        io->iouh_Actual = query_device(io);
        io->iouh_State = drv.state;
        break;

    case UHCMD_USBOPER:                 /* = CMD_START */
    case UHCMD_USBRESUME:
        drv.state = UHSF_OPERATIONAL;
        io->iouh_State = drv.state;
        io->iouh_Flags = (UWORD)((io->iouh_Flags & ~3) | UHFF_HIGHSPEED);
        io->iouh_Actual = 0;
        break;

    case UHCMD_USBSUSPEND:              /* = CMD_STOP */
        drv.state = UHSF_SUSPENDED;
        io->iouh_State = drv.state;
        io->iouh_Actual = 0;
        break;

    case CMD_FLUSH:
        io->iouh_Actual = 0;
        break;

    case B3CMD_GETSTATS:                /* B3Diag */
        drv.st.version = B3STATS_VERSION;
        drv.st.drv_version = DEV_VERSION;
        drv.st.drv_revision = DEV_REVISION;
        drv.st.port_speed = drv.port_speed;
        drv.st.rh_addr = drv.rh_addr;
        {
            ULONG n = sizeof(struct B3Stats);
            if (n > io->iouh_Length)
                n = io->iouh_Length;
            if (n && io->iouh_Data)
                CopyMem(&drv.st, io->iouh_Data, n);
            io->iouh_Actual = n;
        }
        break;

    case B3CMD_CLEARSTATS:
        {
            UBYTE *p = (UBYTE *)&drv.st;
            ULONG i;
            ULONG on = drv.st.irq_enabled, pon = drv.st.pe_on;
            for (i = 0; i < sizeof(struct B3Stats); i++)
                p[i] = 0;
            drv.st.irq_enabled = on;
            drv.st.pe_on = pon;
            drv.st.clear_ms = hw_ms_now();
        }
        io->iouh_Actual = 0;
        break;

    case CMD_RESET:
    case UHCMD_USBRESET:
    case UHCMD_CONTROLXFER:
    case UHCMD_INTXFER:
    case UHCMD_BULKXFER:
    case UHCMD_ISOXFER:
        io->iouh_Actual = 0;
        io->iouh_ExtError = 0;
        io->iouh_Req.io_Flags &= ~IOF_QUICK;
        PutMsg(drv.port, &io->iouh_Req.io_Message);
        return;

    default:
        io->iouh_Req.io_Error = IOERR_NOCMD;
        break;
    }

    if (!(io->iouh_Req.io_Flags & IOF_QUICK))
        ReplyMsg(&io->iouh_Req.io_Message);
    else
        io->iouh_Req.io_Message.mn_Node.ln_Type = NT_REPLYMSG;
}

LONG dev_abortio(struct DevBase *base, struct IOUsbHWReq *io)
{
    (void)base;
    Disable();
    if (io->iouh_Req.io_Message.mn_Node.ln_Type == NT_MESSAGE) {
        if (pe_abort(io)) {
            /* the periodic engine replies */
        } else if (io != drv.current) {
            /* still waiting in the port or one of our queues */
            Remove(&io->iouh_Req.io_Message.mn_Node);
            io->iouh_Req.io_Error = IOERR_ABORTED;
            ReplyMsg(&io->iouh_Req.io_Message);
        } else {
            /* the unit task finishes the running step and replies */
            drv.abort_current = TRUE;
        }
    }
    Enable();
    return 0;
}
