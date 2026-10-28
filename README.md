# berrypi3ap.device

**USB-2.0-Host-Treiber für Poseidon auf PiStorm / Emu68 mit Raspberry Pi 3A+**
*USB 2.0 host controller driver for Poseidon on PiStorm / Emu68 with a Raspberry Pi 3A+*

Version **105.1** · Lizenz / License: **GPL-2.0-or-later** · © 2026 MichiB210

[Deutsch](#deutsch) · [English](#english)

---

## Deutsch

### Was ist das?
`berrypi3ap.device` ist der Treiber zwischen dem USB-Stack **Poseidon** und dem USB-Controller (DWC2) des **Raspberry Pi 3A+** im **PiStorm** unter **Emu68**. Damit funktionieren am USB-Anschluss des Pi zum Beispiel Maus, Tastatur, USB-Sticks und USB-Hubs.

### Was er kann
- **Maus, Tastatur, Hubs, Sticks** – direkt am Pi oder hinter einem USB-2.0-Hub (auch Low-/Full-Speed-Geräte hinter dem Hub).
- **Schnell:** 82,9 MB auf einen Stick in etwa 15 s (direkt am Pi).
- **Flüssige Maus, auch beim Kopieren:** Maus-, Tastatur- und Hub-Abfragen laufen auf einem eigenen USB-Kanal direkt im Interrupt. Gemessen: keine Abfrage später als etwa 10 ms, auch während großer Kopiervorgänge.
- **Interrupt-Betrieb:** Der Treiber wird vom Systemtimer des Pi geweckt, nicht mehr ständig über timer.device.
- **Sauberer Neustart:** Bei Ctrl-Amiga-Amiga und bei Software-Neustarts werden die Interrupts vorher abgeschaltet.
- **Diagnose-Werkzeug `B3Diag`** mit Zählern für Abfragen, Fehler, Splits, Zeitverhalten und Interrupts.

### Voraussetzungen
- PiStorm mit **Raspberry Pi 3A+** und **Emu68**
- **AmigaOS 3.2.x oder 3.9** (andere Versionen ungetestet)
- **Poseidon** (getestet mit 4.5)
- 68020 oder höher (Emu68 erfüllt das)

### Installation
1. `berrypi3ap.device` nach **`DEVS:USBHardware/`** kopieren.
2. `B3Diag` nach **`C:`** kopieren (optional, für die Diagnose).
3. In **Trident** unter *Hardware* den Treiber `berrypi3ap.device`, Unit 0, eintragen bzw. auswählen, speichern und Poseidon neu starten.
4. Prüfen: `version DEVS:USBHardware/berrypi3ap.device` muss **105.1** zeigen.

### Wichtige Zusatz-Software

**Tastatur: Tastenwiederholung („aaaa“, Del gedrückt halten)**
Die hid.class von Poseidon 4.5 erzeugt unter neueren OS-Versionen keine Tastenwiederholung – mit keinem USB-Treiber. Abhilfe ist die aktualisierte **hid.class V4.4** von individual Computers:
- Download: [RapidRoad-Wiki → Software → HIDclassV44.lha](https://wiki.icomp.de/wiki/RapidRoad)
- Alte Datei sichern: `copy SYS:Classes/USB/hid.class SYS:Classes/USB/hid.class.alt`
- Neue `hid.class` nach `SYS:Classes/USB/` kopieren, neu starten.
- Hinweis: **Num-Lock** gibt es auf dem Amiga nicht; der Ziffernblock liefert immer Ziffern.

**NTFS-formatierte Sticks und Festplatten**
Poseidon bindet den Stick ein, lesen kann AmigaOS NTFS aber nur mit einem passenden Dateisystem: **NTFS-3G** für AmigaOS (benötigt filesysbox.library):
- Aminet: [`disk/misc/ntfs3g.m68k-amigaos`](https://aminet.net/package/disk/misc/ntfs3g.m68k-amigaos)
- Quellen: [github.com/salass00/ntfs-3g](https://github.com/salass00/ntfs-3g), [github.com/salass00/filesysbox](https://github.com/salass00/filesysbox)
- Installation laut Anleitung im Archiv.

**Netzwerk (LAN) – in Vorbereitung**
USB-LAN-Adapter mit Realtek **RTL8152B** bieten eine zweite USB-Konfiguration „CDC Ethernet“. Dafür gibt es die **cdceth.class** (SANA-II-Treiber `usbcdceth.device`) in **Poseidon 6.x** von AmigaPorts: [github.com/AmigaPorts/poseidon](https://github.com/AmigaPorts/poseidon). Sie benötigt die poseidon.library ab Version 6 und läuft daher nicht mit Poseidon 4.5. Getestet mit diesem Treiber ist das noch nicht.

### Diagnose mit B3Diag
```
B3Diag CLEAN      ; Zähler löschen (auch CLEAR oder RESET)
...               ; testen: Maus bewegen, kopieren, ...
B3Diag            ; Ergebnis anzeigen
```
Wichtige Zeilen: *poll late* (wie pünktlich Maus/Tastatur abgefragt werden), *Split* (Geräte hinter dem Hub), *Periodic engine* (Abfragen im Interrupt: `stuck` und `spurious` sollten 0 sein), *Interrupt wake-up* (`missed` sollte 0 sein).
Bei Problemen bitte die B3Diag-Ausgabe mitschicken.

### Bekannte Grenzen
- **Keine isochronen Geräte** (z. B. USB-Audio, Webcams).
- **Low-Speed-Maus direkt am Pi:** B3Diag zählt „toggle errors“. Das sind sehr wahrscheinlich Paketwiederholungen der Maus, die korrekt verworfen werden; die Maus funktioniert normal. Hinter einem Hub tritt das nicht auf.
- Low-Speed-Gerät hinter einem **Full-Speed**-Hub wird abgelehnt (Grenze der Hardware, auch unter Linux).
- Feste Adressen für die Pi-Hardware unter Emu68 (noch nicht über `devicetree.resource`).
- Nur Raspberry Pi 3A+ getestet.

### Selbst bauen
```
PATH=<toolchain>/bin:$PATH ./build.sh
```
Benötigt `m68k-amigaos-gcc` (bebbo-Toolchain, getestet mit GCC 16.2). Ergebnis: `berrypi3ap.device` und `B3Diag`.

### Aufbau
| Datei | Inhalt |
|---|---|
| `src/device.c` | Exec-Device, Warteschlangen, Treiber-Task (Kanal 0: Control und Bulk) |
| `src/xfer.c` | Übertragungen; *Periodic Engine* (Kanal 1, Interrupt-Abfragen im Interrupt) |
| `src/channel.c` | DWC2-Kanäle programmieren |
| `src/irq.c` | Interrupt-Server (Emu68 EXTER), Pi-Systemtimer, Reset-Schutz |
| `src/hw.c`, `src/regs.h` | Controller-Initialisierung, Port, DMA, Register |
| `src/rh.c` | virtueller Root-Hub |
| `tools/B3Diag.c` | Diagnose-Werkzeug |

### Herkunft
- Hardware-Wissen (Adressen unter Emu68, Mailbox, Init-Werte, Root-Hub, Split-Abläufe): eigene Vorgängerversionen.
- Abläufe und Fehlerregeln nach dem Linux-Treiber `drivers/usb/dwc2` (GPL-2.0+ OR BSD-3-Clause), eigenständig umgesetzt; Fundstellen sind im Code vermerkt.
- Der Interrupt-Weg unter Emu68 wurde aus den Emu68-Quellen ([michalsc/Emu68](https://github.com/michalsc/Emu68)) nachvollzogen; daraus wurde kein Code übernommen.

Die Assembler-Vorgängerversionen sind in der Git-Historie dieses Repositorys erhalten.

---

## English

### What is it?
`berrypi3ap.device` is the host controller driver between the **Poseidon** USB stack and the DWC2 USB controller of the **Raspberry Pi 3A+** in a **PiStorm** running **Emu68**. It lets you use mice, keyboards, USB sticks and hubs on the Pi's USB port.

### Features
- **Mouse, keyboard, hubs, mass storage** – directly on the Pi or behind a USB 2.0 hub (including low/full-speed devices behind the hub via split transactions).
- **Fast:** 82.9 MB to a stick in about 15 s (directly on the Pi).
- **Smooth mouse, even while copying:** interrupt endpoints (mouse, keyboard, hub status) are polled on their own host channel, entirely inside the interrupt. Measured: no poll later than about 10 ms, even during large copies.
- **Interrupt driven:** the driver task is woken by the Pi system timer instead of polling through timer.device.
- **Clean reboot:** interrupts are switched off on Ctrl-Amiga-Amiga and on software reboots.
- **`B3Diag`** diagnostic tool with counters for polls, errors, splits, timing and interrupts.

### Requirements
- PiStorm with **Raspberry Pi 3A+** and **Emu68**
- **AmigaOS 3.2.x or 3.9** (others untested)
- **Poseidon** (tested with 4.5)

### Installation
1. Copy `berrypi3ap.device` to **`DEVS:USBHardware/`**.
2. Copy `B3Diag` to **`C:`** (optional).
3. In **Trident** → *Hardware*, select `berrypi3ap.device`, unit 0, save and restart Poseidon.
4. Check: `version DEVS:USBHardware/berrypi3ap.device` shows **105.1**.

### Additional software
- **Keyboard repeat:** Poseidon 4.5's hid.class does not generate key repeat on newer OS versions, with any USB driver. Install the updated **hid.class V4.4** from individual Computers ([RapidRoad wiki](https://wiki.icomp.de/wiki/RapidRoad), *HIDclassV44.lha*) into `SYS:Classes/USB/`. The Amiga has no Num Lock.
- **NTFS media:** use **NTFS-3G** for AmigaOS (needs filesysbox.library): Aminet [`disk/misc/ntfs3g.m68k-amigaos`](https://aminet.net/package/disk/misc/ntfs3g.m68k-amigaos), sources at [salass00/ntfs-3g](https://github.com/salass00/ntfs-3g).
- **Network (in preparation):** RTL8152B adapters offer a second "CDC Ethernet" configuration, handled by **cdceth.class** (`usbcdceth.device`) from **Poseidon 6.x** ([AmigaPorts/poseidon](https://github.com/AmigaPorts/poseidon)), which requires poseidon.library v6+. Not yet tested with this driver.

### Diagnostics
`B3Diag CLEAN` (or CLEAR / RESET) resets the counters, then run your test, then `B3Diag` shows the results. Please include the output when reporting problems.

### Known limitations
- No isochronous transfers (USB audio, webcams).
- Low-speed mouse directly on the Pi: B3Diag counts "toggle errors" (most likely retransmitted packets that are correctly discarded); the mouse works normally. Not seen behind a hub.
- Low-speed devices behind a full-speed hub are refused (hardware limit, as in Linux).
- Fixed Emu68 addresses for the Pi hardware (not yet read from `devicetree.resource`).
- Tested on a Raspberry Pi 3A+ only.

### Building
`PATH=<toolchain>/bin:$PATH ./build.sh` with the bebbo `m68k-amigaos-gcc` toolchain (tested with GCC 16.2).

### Credits / origin
Own hardware knowledge from the earlier assembler versions (kept in this repository's history); transfer and error handling modelled on Linux `drivers/usb/dwc2` (GPL-2.0+ OR BSD-3-Clause), implemented independently with references in the code; the Emu68 interrupt path was understood by reading the [Emu68](https://github.com/michalsc/Emu68) sources (no code taken).
