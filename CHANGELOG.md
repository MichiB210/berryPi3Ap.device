# Changelog

## 105.1 (28.10.2026)
- Waiting for microframe 7 before a periodic start-split: sleep into microframe 6, then wait the rest (≤125 µs). About 1300 instead of 2100 interrupts per second at the same punctuality.

## 105.0 (28.10.2026)
- **Periodic engine:** interrupt endpoints (mouse, keyboard, hub status) are polled on DWC2 channel 1 entirely inside the interrupt (system timer C1 for due times, channel-halt interrupt for SSPLIT → CSPLIT → result, reply from the interrupt). Control and bulk stay on channel 0 in the task.
- While copying: no poll later than about 10 ms (104.4: up to about 70 ms).
- Watchdog for a stuck poll; automatic fallback to task polling if the interrupt is switched off.

## 104.4
- Task wake-up by the Pi system timer (compare C3) instead of timer.device; timer.device only as a 50 ms safety net.

## 104.3
- Interrupt path under Emu68 (level 6 EXTER server, Pi legacy interrupt controller). Reboot protection: keyboard reset handler, ColdReboot() patch, interrupt switched off at init; storm guard.

## 104.2
- Timer set to the next due poll instead of a fixed 1 ms (−72 % wake-ups).

## 104.1
- Unit task priority 15 (was 5).

## 104.0
- Timing statistics in B3Diag (longest control/bulk/interrupt step, timer lateness).

## 103.7
- A transfer that completes while the channel is being halted is no longer lost.

## 103.3 – 103.6
- B3Diag statistics tool; tuning of periodic split timing.

## 103.1
- Fixed a busy loop when a device was attached (port signal not cleared). 82.9 MB in 14 s directly on the Pi.

## 103.0
- Complete rewrite in C (stage 1): queues and a unit task instead of a global lock, bulk in time slices, toggles per endpoint, control transfers of any length, bulk through split transactions, Linux dwc2 rules.

## Before 103
- Assembler versions (USB55 … USB102); see the Git history.
