#!/bin/sh
# Build berrypi3ap.device with m68k-amigaos-gcc (bebbo toolchain).
# "sh build.sh rom" builds berrypi3ap_rom.device instead, the Kickstart
# module: RTF_COLDSTART, no data/BSS hunk (see ROMBUILD in dev.h).
# -fno-peephole2 there: gcc 6.5 m68k peephole2 has produced wrong code.
set -e
cd "$(dirname "$0")/src"
GCC=${GCC:-m68k-amigaos-gcc}
CFLAGS="-O2 -m68020 -fomit-frame-pointer -fno-builtin -ffreestanding -fno-strict-aliasing -noixemul -Wall -Wextra -Wno-unused-parameter"
OUT=berrypi3ap.device
O=o
if [ "$1" = rom ]; then
    CFLAGS="$CFLAGS -DROMBUILD -ffixed-a4 -fno-peephole2"
    OUT=berrypi3ap_rom.device
    O=rom.o
fi
$GCC $CFLAGS -c start.S -o start.$O
for f in device hw channel xfer rh irq util; do
    $GCC $CFLAGS -c $f.c -o $f.$O
done
$GCC -nostdlib -nostartfiles -noixemul start.$O device.$O hw.$O channel.$O xfer.$O rh.$O irq.$O util.$O -lgcc -o ../$OUT
echo "built ../$OUT"
[ "$1" = rom ] && exit 0
cd ../tools
$GCC -O2 -m68020 -noixemul -Wall -o ../B3Diag B3Diag.c
echo "built ../B3Diag"
