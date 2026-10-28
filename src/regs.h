/*
 * berrypi3ap.device - USB host controller driver for PiStorm/Emu68 (Pi 3A+)
 * Copyright (C) 2026 MichiB210
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * DWC2 register map.  Offsets and bit positions are hardware facts; they
 * were found experimentally in USB55..USB101 and cross-checked against the
 * Synopsys/Linux dwc2 hw.h (GPL-2.0+ OR BSD-3-Clause).
 */
#ifndef REGS_H
#define REGS_H

/* Emu68 maps the Pi peripherals for the 68k side (USB57..USB102). */
#define USB_BASE        0xF2980000UL
#define MBOX_BASE       0xF200B880UL

/* Core global registers */
#define GAHBCFG         0x008
#define GUSBCFG         0x00C
#define GRSTCTL         0x010
#define GINTSTS         0x014
#define GINTMSK         0x018
#define GRXFSIZ         0x024
#define GNPTXFSIZ       0x028
#define GNPTXSTS        0x02C
#define GSNPSID         0x040
#define GHWCFG2         0x048
#define HPTXFSIZ        0x100

/* Host registers */
#define HCFG            0x400
#define HFIR            0x404
#define HFNUM           0x408
#define HPTXSTS         0x410
#define HAINT           0x414
#define HAINTMSK        0x418
#define HPRT0           0x440
#define PCGCTL          0xE00

/* Host channel n registers */
#define HCCHAR(n)       (0x500 + (n) * 0x20)
#define HCSPLT(n)       (0x504 + (n) * 0x20)
#define HCINT(n)        (0x508 + (n) * 0x20)
#define HCINTMSK(n)     (0x50C + (n) * 0x20)
#define HCTSIZ(n)       (0x510 + (n) * 0x20)
#define HCDMA(n)        (0x514 + (n) * 0x20)

/* GAHBCFG: BCM2835 value 0x10 (Linux dwc2_set_bcm_params) + DMA enable */
#define GAHBCFG_BCM_AXI 0x00000010UL
#define GAHBCFG_DMA_EN  0x00000020UL

/* GUSBCFG */
#define GUSBCFG_FORCEHOST 0x20000000UL
#define GUSBCFG_FORCEDEV  0x40000000UL

/* GRSTCTL */
#define GRSTCTL_CSFTRST     0x00000001UL
#define GRSTCTL_RXFFLSH     0x00000010UL
#define GRSTCTL_TXFFLSH     0x00000020UL
#define GRSTCTL_TXFNUM_ALL  0x00000400UL
#define GRSTCTL_AHBIDLE     0x80000000UL

/* HPRT0 */
#define HPRT_CONN       0x00000001UL
#define HPRT_CONNDET    0x00000002UL    /* W1C */
#define HPRT_ENA        0x00000004UL    /* W1C: writing 1 disables! */
#define HPRT_ENACHG     0x00000008UL    /* W1C */
#define HPRT_OVRCUR     0x00000010UL
#define HPRT_OVRCHG     0x00000020UL    /* W1C */
#define HPRT_RES        0x00000040UL
#define HPRT_SUSP       0x00000080UL
#define HPRT_RST        0x00000100UL
#define HPRT_PWR        0x00001000UL
#define HPRT_SPD_MASK   0x00060000UL
#define HPRT_SPD_HIGH   0x00000000UL
#define HPRT_SPD_FULL   0x00020000UL
#define HPRT_SPD_LOW    0x00040000UL
/* bits that must be written as 0 to avoid clearing/disabling (USB57) */
#define HPRT_W1C_MASK   (HPRT_CONNDET | HPRT_ENA | HPRT_ENACHG | HPRT_OVRCHG)

/* GINTSTS / GINTMSK */
#define GINT_HCHINT     0x02000000UL    /* a host channel interrupt */

/* HFNUM: bits 31:16 frame time remaining (PHY clocks) */
#define HFNUM_FRREM(v)  (((v) >> 16) & 0xFFFFUL)

/* HCCHAR */
#define HCCHAR_MPS(x)       ((ULONG)(x) & 0x7FF)
#define HCCHAR_EPNUM(x)     (((ULONG)(x) & 0xF) << 11)
#define HCCHAR_EPDIR_IN     0x00008000UL
#define HCCHAR_LSDEV        0x00020000UL
#define HCCHAR_EPTYPE(x)    (((ULONG)(x) & 3) << 18)
#define HCCHAR_MC(x)        (((ULONG)(x) & 3) << 20)
#define HCCHAR_DEVADDR(x)   (((ULONG)(x) & 0x7F) << 22)
#define HCCHAR_ODDFRM       0x20000000UL
#define HCCHAR_CHDIS        0x40000000UL
#define HCCHAR_CHENA        0x80000000UL

#define EPTYPE_CONTROL  0
#define EPTYPE_ISOC     1
#define EPTYPE_BULK     2
#define EPTYPE_INTR     3

/* HCSPLT */
#define HCSPLT_PRTADDR(x)   ((ULONG)(x) & 0x7F)
#define HCSPLT_HUBADDR(x)   (((ULONG)(x) & 0x7F) << 7)
#define HCSPLT_XACTPOS_ALL  0x0000C000UL
#define HCSPLT_COMPSPLT     0x00010000UL
#define HCSPLT_SPLTENA      0x80000000UL

/* HCINT */
#define HCINT_XFERCOMP  0x00000001UL
#define HCINT_CHHLTD    0x00000002UL
#define HCINT_AHBERR    0x00000004UL
#define HCINT_STALL     0x00000008UL
#define HCINT_NAK       0x00000010UL
#define HCINT_ACK       0x00000020UL
#define HCINT_NYET      0x00000040UL
#define HCINT_XACTERR   0x00000080UL
#define HCINT_BBLERR    0x00000100UL
#define HCINT_FRMOVRUN  0x00000200UL
#define HCINT_DATATGLERR 0x00000400UL
#define HCINT_ERRORS    (HCINT_AHBERR | HCINT_STALL | HCINT_XACTERR | \
                         HCINT_BBLERR | HCINT_FRMOVRUN | HCINT_DATATGLERR)
/* software-only marker: channel did not halt in time */
#define HCINT_SW_TIMEOUT 0x80000000UL

/* HCTSIZ */
#define HCTSIZ_XFERSIZE(x)  ((ULONG)(x) & 0x7FFFF)
#define HCTSIZ_PKTCNT(x)    (((ULONG)(x) & 0x3FF) << 19)
#define HCTSIZ_GET_PKTCNT(v) (((v) >> 19) & 0x3FF)
#define HCTSIZ_PID_MASK     0x60000000UL
#define HCTSIZ_PID_DATA0    0x00000000UL
#define HCTSIZ_PID_DATA2    0x20000000UL
#define HCTSIZ_PID_DATA1    0x40000000UL
#define HCTSIZ_PID_SETUP    0x60000000UL

/* Per-channel transfer limits for the BCM283x core (Linux dwc2 params.c,
 * dwc2_set_bcm_params): 65535 bytes, 511 packets.  Verified in USB102. */
#define CHAN_MAX_BYTES  65535UL
#define CHAN_MAX_PKTS   511UL

/* Mailbox (offsets from MBOX_BASE) */
#define MBOX_READ       0x00
#define MBOX_STATUS     0x18
#define MBOX_WRITE      0x20
#define MBOX_FULL       0x80000000UL
#define MBOX_EMPTY      0x40000000UL
#define MBOX_CH_PROP    8
#define TAG_SET_POWER   0x00028001UL
#define POWER_DEV_USB   3

#endif
