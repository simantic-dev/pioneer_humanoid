#!/bin/sh
# Sim addition. The Pioneer joint closing its loop on a MuJoCo arm (sim/mujoco/arm.xml), in lockstep.
# Needs: `sim` on PATH, sim/build/joint-arm.elf from `ARM=1 sh sim/build.sh`, and a Python with
# mujoco, numpy and matplotlib: VENV=<venv dir> (python -m venv $VENV; $VENV/bin/pip install mujoco numpy matplotlib).
# sim embeds the same Python version as the venv's, so PYTHONPATH points at the venv's site-packages.
# Output: sim/out/arm-sim.log (ARM t=... every 10 ms), sim/screenshots/arm-step.png; exit 1 unless SETTLED.
set -e
cd "$(dirname "$0")/.."
: "${VENV:?set VENV to a venv with mujoco installed}"
SP=$(ls -d "$VENV"/lib/python3.*/site-packages)
mkdir -p sim/out
# ENC_OFFSET_RAD: the encoder reads pi at the hang pose, matching SIM_ZERO in sim/arm-position.patch
ENC_OFFSET_RAD=3.14159 PYTHONPATH=$SP "${SIM:-sim}" run --scenario sim/scenario-arm.yaml --usb-cdc-console dut.usb --timeout 11 --ascii \
  --frames sim/out/arm-frames.txt --expect "Motor ready!!!" > sim/out/arm-uart.log 2> sim/out/arm-sim.log
"$VENV/bin/python" sim/plot-arm.py | tee sim/out/arm-result.txt
grep -q '^SETTLED' sim/out/arm-result.txt
