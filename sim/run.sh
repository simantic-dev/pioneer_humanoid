#!/bin/sh
# Sim addition. Runs the joint (sim/scenario.yaml) for N virtual seconds (default 3.5); needs `sim` on
# PATH (or SIM=...) and sim/build/joint.elf from sim/build.sh. Fails (exit 1) unless the firmware's
# own console shows the FOC alignment passing and the peers report the joint tracked the CAN command.
# Output: sim/out/uart.log (the joint's USB CDC console), sim/out/sim.log (peer logs, warnings),
# sim/out/can-frames.txt (every CAN frame and SPI transfer).
set -e
cd "$(dirname "$0")/.."
SIM=${SIM:-sim}
[ -f sim/build/joint.elf ] || { echo "build first: sim/build.sh"; exit 1; }
mkdir -p sim/out
"$SIM" run --scenario sim/scenario.yaml --usb-cdc-console dut.usb --timeout "${1:-3.5}" --ascii \
  --frames sim/out/can-frames.txt \
  --expect "STM32 Serial OK!" --expect "MOT:sensor dir: CW" --expect "MOT:PP check: OK!" \
  --expect "Motor ready!!!" --expect-absent "ERR-MOT:Init FOC fail" --expect-absent "Motor init failed" \
  > sim/out/uart.log 2> sim/out/sim.log
grep -a -q "joint tracked the CAN command" sim/out/sim.log || { echo "FAIL: joint did not track the CAN command"; exit 1; }
grep -a -E "(canpeer|encoder)\]" sim/out/sim.log
echo "PASS   logs in sim/out/"
