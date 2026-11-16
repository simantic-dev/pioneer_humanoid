#!/bin/sh
# Sim addition. Builds the bootloader + FreeRTOS app (PlatformIO envs bootloader and app, default env
# `app`) into sim/build/boot.elf. Needs PlatformIO. Build-only changes, restored afterwards: foc.cpp is
# dropped from the app's source filter (it includes Arduino headers inside the stm32cube env) and
# sim/firmware-console.patch adds an LPUART1 console (their code prints nothing).
set -e
cd "$(dirname "$0")/../src/embedded/STM32"
cp platformio.ini platformio.ini.orig; cp app/src/firmware.c firmware.c.orig
trap 'mv platformio.ini.orig platformio.ini; mv firmware.c.orig app/src/firmware.c; rm -f app/src/firmware.c.rej *.map' EXIT
patch -p0 app/src/firmware.c < ../../../sim/firmware-console.patch
sed -i.bak 's|^build_src_filter = +<../app/> -<\*>$|& -<../app/src/foc.cpp>|' platformio.ini; rm -f platformio.ini.bak
pio run -e bootloader
python3 pad_bootloader.py
# app/src/bootloader.S .incbin's ../.pio/build/bootloader/bootloader.bin relative to the project's parent
mkdir -p ../.pio/build/bootloader && cp .pio/build/bootloader/bootloader.bin ../.pio/build/bootloader/
pio run -e app
mkdir -p ../../../sim/build
cp .pio/build/app/app.elf ../../../sim/build/boot.elf
