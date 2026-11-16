#!/bin/sh
# Sim addition. Boots sim/build/boot.elf (sim/build-boot.sh) for 5 virtual seconds; fails unless the
# app's LPUART1 console shows the FreeRTOS task running. Output: sim/out/boot-uart.log, boot-sim.log.
set -e
cd "$(dirname "$0")/.."
SIM=${SIM:-sim}
[ -f sim/build/boot.elf ] || { echo "build first: sim/build-boot.sh"; exit 1; }
mkdir -p sim/out
"$SIM" STM32G474RET6 run --elf sim/build/boot.elf --ascii --timeout 5 \
  --expect "PIONEER: app up" --expect "tick 4" --expect "RESULT: PASS" \
  > sim/out/boot-uart.log 2> sim/out/boot-sim.log
cat sim/out/boot-uart.log; grep -a "PLL out of range" sim/out/boot-sim.log | cut -c1-260
