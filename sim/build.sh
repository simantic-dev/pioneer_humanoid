#!/bin/sh
# Sim addition. Builds the joint image (PlatformIO env nucleo_g474re) into sim/build/joint.elf.
# The env does not build as committed; three build-only fixes are applied to the working tree and
# the downloaded libraries (see the README "Changes to the firmware"). Needs PlatformIO.
set -e
cd "$(dirname "$0")/../src/embedded/STM32"
cp platformio.ini platformio.ini.orig
# ZERO0=1 (with ARM=1): the encoder's angle at the hang pose is 0 instead of pi (SIM_ZERO), for sim/mujoco/cases.py's wrap case.
# ARM=1 (sim/mujoco): position-control build for the MuJoCo arm; see sim/arm-position.patch. The default build is unchanged.
if [ -n "$ARM" ]; then
  patch -p4 < ../../../sim/arm-position.patch
  sed -i.bak 's/HSE_VALUE=4800000U/HSE_VALUE=24000000U/' platformio.ini  # the 24 MHz crystal sim/scenario.yaml declares
fi
sed -i.bak 's/^lib_archive = false$/lib_archive = false\nlib_ignore = FreeRTOS-Kernel/' platformio.ini
pio pkg install -e nucleo_g474re
sed -i.bak 's/enableTimerClock(handle);/enableTimerClock(handle->Instance);/' ".pio/libdeps/nucleo_g474re/Simple FOC/src/drivers/hardware_specific/stm32/stm32_mcu.cpp"
sed -i.bak 's/enableTimerClock(&encoder_handle)/enableTimerClock(encoder_handle.Instance)/' .pio/libdeps/nucleo_g474re/SimpleFOCDrivers/src/encoders/stm32hwencoder/STM32HWEncoder.cpp
PLATFORMIO_BUILD_FLAGS="-include stm32g4xx_hal.h${ZERO0:+ -DSIM_ZERO=0.0f}" pio run -e nucleo_g474re
mkdir -p ../../../sim/build
cp .pio/build/nucleo_g474re/firmware.elf ../../../sim/build/joint${ARM:+-arm}${ZERO0:+-zero0}.elf
mv platformio.ini.orig platformio.ini; rm -f platformio.ini.bak  # the committed platformio.ini is unchanged
if [ -n "$ARM" ]; then git checkout src/main.cpp; fi
