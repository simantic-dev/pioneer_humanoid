## Demo: the joint firmware in simulation

```sh
./demo          # after ./demo build (PlatformIO, once); about 30 s
```

![joint firmware demo](media/pioneer-joint.gif)

```mermaid
flowchart LR
  fw["STM32G474RET6 joint firmware<br/>src/embedded/STM32, unmodified"]
  plant["plant peer<br/>sim/peers/foc_plant.py + pmsm.py"]
  enc["encoder peer<br/>sim/peers/mt6835_peer.py"]
  can["CAN master peer<br/>sim/peers/can_peer.py"]
  host["terminal (./demo)"]
  fw -- "TIM4 3-phase PWM duty" --> plant
  plant -- "shaft angle" --> enc
  enc -- "SPI1 burst read: angle, status, CRC8" --> fw
  can -- "CAN 0x123 target '32' (8.00 rad/s)" --> fw
  fw -- "CAN 0x123 status '67890' every 100 ms" --> can
  fw -- "USB CDC console: MOT:* lines, encoder angle" --> host
```

The joint-board firmware from `src/embedded/STM32` runs unmodified on a simulated STM32G474RET6: its SimpleFOC
alignment, the MT6835 encoder read over SPI, and its CAN status frames. The motor is a Python PMSM stand-in, not a
CubeMars model, and the CAN master sends this firmware's own ASCII target on 0x123, not the MIT protocol in
`src/interfacing`. The peers work at transaction level (SPI bytes, CAN frames) with no electrical effects. Known
gaps: the velocity loop does not settle at the 8.00 rad/s target in this model (the shaft runs near 25 rad/s before
the command and 40 rad/s after), the firmware reports `No current sense`, and the peer's "followed" check is a
speed ratio, not tracking accuracy. Prerequisites: `sim` 0.10.0 (`curl -fsSL https://simantic.com/install | sh`) and
PlatformIO for `./demo build`.

## This fork: the Pioneer joint board in a simulator

This is Simantic's fork of WATonomous' Pioneer humanoid repository, kept as a simulation showcase. It runs
the joint-board firmware in `src/embedded/STM32` (STM32G474RE, SimpleFOC, MT6835 encoder over SPI, CAN)
on a simulated STM32G474RET6, with no board attached. The fork adds a `sim/` directory and this section;
the committed firmware sources are unchanged (the build-time changes are listed below). Nothing here is
offered back or posted upstream. If you are on the team and want this removed, say so and it goes.

```
joint firmware's own USB console, 3.5 s of virtual time (sim/run.sh)
  [0.301367s] MOT:Align sensor.
  [0.742186s] MOT:sensor dir: CW
  [0.742206s] MOT:PP check: OK!
  [0.886186s] MOT:Zero elec. angle: 6.27
  [0.926203s] ERR-MOT:No current sense.
  [1.126208s] Motor ready!!!
  [3.499977s] 64.49            <- encoder.getAngle(), rad
CAN master                      shaft 25.0 rad/s before command, 101 status frames
                                sent target '32' on 0x123
                                shaft 39.7 rad/s after command, ratio 1.59, 115 status frames
```

![Encoder angle and shaft speed](sim/screenshots/encoder-and-speed.png)

Top: the angle the firmware prints from its MT6835 reads (26,279 prints, 64.49 rad at the end). Bottom:
the PMSM stand-in's shaft speed. It settles at 25.0 rad/s, then steps to about 40 rad/s after the master's
command frame at 3.150 s (dashed). Configuration `sim/scenario.yaml`, 3.5 s virtual, Simantic `sim` 0.10.0
(build f219eb31), 2026-11-16; angle from the run's USB console log, speed from the plant peer's own log.

![CAN 0x123 status interval](sim/screenshots/can-0x123.png)

The joint's 0x123 status frames from the run's `--frames` capture: 118 frames, first at 1.126 s, mean
interval 20.21 ms (range 20.05 to 20.54 ms). The firmware sends one every 100 ms of its own `millis()`, see
"Clock" below for why that is 20 ms of virtual time. Same configuration and sim build.

![loop() period](sim/screenshots/loop-timing.png)

Virtual time between consecutive encoder prints, i.e. one pass of `loop()` (SPI read, `loopFOC`, print):
median 87 us, maximum 385 us, with no change at the CAN command. Same configuration and sim build.

![Plant currents and duty](sim/screenshots/foc-state.png)

The plant's dq currents and the TIM4 CH1 duty the firmware writes, over the alignment (0.30 s to 0.93 s) and
the open-loop run. These currents are the stand-in's state, not a firmware measurement: the firmware
reports `ERR-MOT:No current sense.` and has none. Same configuration and sim build.

**What runs:**

| Part | In the simulation |
| --- | --- |
| Joint firmware (`nucleo_g474re` env, `src/main.cpp`) | Arduino core, SimpleFOC `BLDCDriver3PWM` on TIM4, MT6835 on SPI1, FDCAN2, console on USB CDC |
| PMSM plant (`sim/peers/foc_plant.py`) | reads TIM4 duty, integrates dq currents and rotor speed; stand-in, not the real motor |
| MT6835 encoder (`sim/peers/mt6835_peer.py`) | answers the driver's `0xA0 0x03` angle burst read (21-bit angle, status, CRC8) from the plant's shaft |
| CAN master (`sim/peers/can_peer.py`) | receives the joint's frames on std ID 0x123 and sends the `"32"` speed command |
| Bootloader + FreeRTOS app (`bootloader` and `app` envs) | second image, boots on its own, see the table below |

**The scenario**, as `sim graph --scenario sim/scenario.yaml` prints it:

```mermaid
flowchart LR
  dut["dut<br/>STM32G474RET6"]
  dut_canpeer(["canpeer<br/>CAN.ScriptedCanNode"])
  dut_spi1mux(["spi1mux<br/>SPI.SPIMultiplexer"])
  dut -- "spi1" --- dut_spi1mux
  dut_plant(["plant<br/>Scripted.ScriptedPeripheral"])
  dut -- "sysbus" --- dut_plant
  dut_encoder(["encoder<br/>SPI.ScriptedSpiSlave"])
  dut -- "spi1mux 0x0" --- dut_encoder
  medium0{{"can"}}
  dut -- "fdcan2" --- medium0
  dut_canpeer --- medium0
```

**Configurations tried** (all with `sim` 0.10.0, build f219eb31, 2026-11-16; wall times on a loaded laptop):

| Configuration | Where | Virtual | Wall | Result |
| --- | --- | --- | --- | --- |
| Joint, HSE 24 MHz (the NUCLEO crystal) | `sim/scenario.yaml`, `sim/run.sh` | 3.5 s | 22 to 26 s | runs: alignment passes (`sensor dir: CW`, `PP check: OK!`), `Motor ready!!!` at 1.126 s, 118 status frames, shaft follows the CAN command (ratio 1.59) |
| Joint, HSE 4.8 MHz (the `HSE_VALUE` in their `platformio.ini`) | `sim/scenario-hse4p8.yaml` | 3.5 s | 2.8 s | does not run: the USB console prints nothing, no CAN status frame is sent, the shaft stays at 0.0 rad/s |
| Bootloader + FreeRTOS app, console patch | `sim/build-boot.sh`, `sim/run-boot.sh` | 5 s | 0.9 s | runs: `PIONEER: app up` at 1.37 ms, ticks 1.006 s apart, `RESULT: PASS` at 4.013 s |

The joint image is `firmware.elf` from `pio run -e nucleo_g474re` (sha256 prefix 1097640619bb639a; a second
build gave the same hash).

**Clock** (from `app/src/system.c`, source reading plus the boot image run; the boot image was not rebuilt).
`SystemClock_Config` uses STM32F4 PLL values on a G4: HSI source, `PLLM = 16`, `PLLN = 336`, `PLLP = DIV4`,
`PLLQ = 7`, and `RCC_OscInitTypeDef` is not zero-initialised, so `PLLR` is whatever was on the stack. On the
STM32G4, PLLN is a 7-bit field (RM0440 section 7.4.4, bits 14:8, legal 8 to 127; HAL's `IS_RCC_PLLN_VALUE` is
also 8 to 127), so `336 << 8` leaves PLLN = 0x50 = 80 and spills into bit 16 (PLLPEN). With `PLLM = 16` on HSI16
the VCO input is 1 MHz (datasheet range 2.66 to 16 MHz) and the VCO is 80 MHz (datasheet minimum 96 MHz): out
of spec. The simulator masks PLLN to 7 bits, warns, and still sets PLLRDY, because a silicon measurement
(NUCLEO-G474RE rev X) locked at 1 MHz / 80 MHz. The app prints `SYSCLK: 13333333`; a PLLR field of 6 would give
80 / 6 = 13.33 MHz, but that PLLR is 6 in this image is `unverified`. HAL, SysTick and `HAL_Delay` agree with
13.33 MHz (ticks land 1.002 s apart). This is a firmware configuration issue rather than a simulator model
issue. A legal example is HSI16 / 4 x 85 / 2 = 170 MHz.

**Corrected (2026-11-16).** An earlier version of this section said the G4 field width was not checked and
framed 13.33 MHz against a 16 MHz expectation as a model question. The field width is now checked against
RM0440, and the 13.33 MHz figure follows from the source configuration.
For the joint image the `HSE_VALUE=4800000U` build flag differs from the NUCLEO's 24 MHz crystal, so
the first scenario declares 24 MHz. `millis()` then runs about 5x fast, consistent with that mismatch: the
0x123 frames come every 20.2 ms of virtual time instead of every 100 ms. The CAN check is therefore a speed ratio (1.45 to 1.75 accepted by
the peer).

**Gaps, with the evidence:**
- **The joint does not run at 4.8 MHz HSE in the simulator** (`sim/scenario-hse4p8.yaml`). The log shows
  the RCC computing a 57.6 MHz VCO; the console, which waits on USB (`while (!Serial)` in `main.cpp`), is
  silent. That USB is the blocker is read from the source, not isolated in a run: `unverified`.
- **`ERR-MOT:No current sense.`** is printed by SimpleFOC at start-up: this firmware has no current sensing.
- **The motor runs open loop** (`motor.controller = velocity_openloop`, `main.cpp:137`): the encoder is read
  and printed but does not close a loop.
- **The CAN protocol is not the DBC and not the CubeMars protocol.** `main.cpp` sends the ASCII bytes
  `67890` on std ID 0x123 and reads two ASCII digits from 0x123 as `25 * NN / 100` rad/s; a search of
  `src/embedded/STM32/src` and `include` finds no mention of CubeMars or the DBC. No CubeMars motor is
  simulated.
- The plant, the encoder's status bits (always 0) and the 0.6 electrical radian start offset are stand-ins
  chosen for the run; the peers mark them `MODELED SIMPLIFICATION`.
- Not run: the ESP32-S3 sources, the Python and ROS modules, silicon.

**Run it** (needs PlatformIO with the ststm32 platform, and Simantic's `sim` on PATH):

```
sim/build.sh        # builds sim/build/joint.elf
sim/run.sh          # 3.5 virtual seconds; logs and frame capture in sim/out/, exits 1 unless the checks hold
sim/build-boot.sh   # builds sim/build/boot.elf (bootloader + app)
sim/run-boot.sh
python3 sim/plot.py "caption"   # redraws sim/screenshots/*.png from sim/out/ (needs matplotlib)
```

**Changes to the firmware: none are committed.** The scripts make these build-time changes and restore the
files afterwards:
- `platformio.ini`: `lib_ignore = FreeRTOS-Kernel` for the `nucleo_g474re` env (the library finder pulls the
  vendored kernel into the Arduino build, which has no `FreeRTOSConfig`); for the `app` env, `foc.cpp` is dropped
  from `build_src_filter` (it includes Arduino headers inside the stm32cube env).
- In the downloaded libraries (`.pio/libdeps`): `enableTimerClock(handle)` becomes `enableTimerClock(handle->Instance)`
  in Simple FOC 2.4.0 and likewise in SimpleFOCDrivers 1.0.9 `STM32HWEncoder.cpp`.
- A build flag `-include stm32g4xx_hal.h`, because `src/Drivers/*.h` include HAL headers without the umbrella header.
- `sim/firmware-console.patch` (boot image only): adds a console on LPUART1 (PA2/PA3) to `app/src/firmware.c`,
  since the stm32cube envs print nothing.
- `pad_bootloader.py` is run by hand and its output copied to `../.pio/build/bootloader/`, where `app/src/bootloader.S` `.incbin`s it.

**Licence.** The repository is Apache-2.0 and `LICENSE` is kept. The files under `sim/` are additions by
Simantic, not by the Pioneer team; the peers, scenarios and scripts there are offered under the same licence.

Everything below is the Pioneer team's README.

---

# Pioneer - UWaterloo's First Humanoid Robot 

We are a team of undergraduate student from UWaterloo building a 160 cm humanoid robot. We are the first engineering design team in Canada building a humanoid!

Our humanoid robot will feature 2 arms + 2 legs and we are also developing a 22 DOF humanoid hand that can be modularly switchable with the arm gripper.

More info at Pioneer's website: https://watonomous.github.io/humanoid-docs/index.html

## Find your subteam

| Subteam | Start here |
|---|---|
| Software: manipulation | [`src/teleop`](src/teleop) · [`src/robot_learning`](src/robot_learning) · [`src/simulation`](src/simulation) |
| Software: locomotion | [`src/simulation/humanoid_rl`](src/simulation/humanoid_rl) · [`src/simulation/humanoid_rl_tasks`](src/simulation/humanoid_rl_tasks) |
| Firmware / interfacing | [`src/embedded`](src/embedded) · [`src/interfacing`](src/interfacing) |
| Perception | [`src/perception`](src/perception) |

## Quick start

`watod` is our Docker Compose wrapper: it spins up one dev container per module (interfacing, perception, simulation, …), so you only run the stack your subteam needs.

```bash
cp watod-config.sh watod-config.local.sh   # set your ACTIVE_MODULES
./watod up -d                 # start your modules
./watod -t <service>          # shell into one (interfacing, perception, simulation_mj, ...)
./watod build <service>       # rebuild after a Dockerfile / dependency change
./watod down                  # stop
```

Your `src/<module>` folder is mounted into the container, so edits on your machine show up inside it immediately without rebuild.

`watod-config.sh` is shared defaults and is CI-guarded, so don't commit personal changes to it. Everything local goes in `watod-config.local.sh` (gitignored).

| `ACTIVE_MODULES` | What it runs |
|------------------|--------------|
| `interfacing` | CAN / hardware interfacing, `joint_command` |
| `perception` | Perception (cameras, GPU), `voxel_grid` |
| `simulation_isaac` | **Isaac Lab 2.3.2** — SO101 robot learning, RL tasks, Quest teleop |
| `simulation_mj` | MuJoCo / mjlab RL |
| `terrain_mapping` | GPU elevation map from D455 bags ([src/terrain_mapping](src/terrain_mapping)) |

**Isaac Lab sim (recommended):** see [docker/simulation/isaac_lab/README.md](docker/simulation/isaac_lab/README.md).

## Repo map

Two workflows share this repo:

- **Real robot:** `embedded` firmware ⇄ `interfacing` (CAN ⇄ ROS 2) ⇄ `perception` — driven live by `teleop`, or by a policy.
- **Sim / learning:** `teleop` collects demos in `simulation` scenes → `robot_learning` records datasets and trains policies, or `simulation/humanoid_rl` trains RL policies → deploy back through `interfacing`.

**`robot_learning` vs RL:** `src/robot_learning` learns policies from recorded datasets (sim or real), e.g. ACT, SmolVLA, pi0.5. RL is sim-based (Isaac Lab), so it lives in `src/simulation/humanoid_rl*`.

```
humanoid
├── watod, watod-config.sh   # container orchestrator + module config
├── modules/  docker/        # compose files + Dockerfiles, per module
├── src/
│   ├── pioneer_humanoid/    # the robot definition (imported everywhere)
│   ├── interfacing/         # CAN ⇄ ROS 2 bridge, real-arm bring-up
│   ├── embedded/            # STM32 / ESP32S3 motor firmware
│   ├── perception/          # cameras, voxel grid
│   ├── terrain_mapping/     # GPU elevation mapping (elevation_mapping_cupy)
│   ├── simulation/          # Isaac Lab + MuJoCo: RL tasks, teleop scenes
│   ├── teleop/              # drive the arm (sim or real)
│   ├── robot_learning/      # record demos, train / eval policies
│   └── common_msgs/         # shared ROS 2 messages
├── assets/                  # robot URDF / USD / meshes, scene props
├── models/                  # frozen RL policy baselines
├── outputs/                 # training runs (gitignored)
└── docs/                    # pointer to the docs site
```

Full detail — [src/simulation/README.md](src/simulation/README.md). Other areas: each has its own `README.md`.

## CAN / arm bring-up

Connect → calibrate → visualize → move, top to bottom: [src/interfacing/README.md](src/interfacing/README.md)

```bash
./src/interfacing/can/scripts/can_udev.sh install   # once per host → /dev/canable
```

## Things to know

- CI does **not** build `simulation_*` or `embedded`. A green check on those means nothing was built, so test them yourself.
- Set a unique `ROS_DOMAIN_ID` (0–232) in `watod-config.local.sh` if someone else runs ROS on the same subnet, or you will see each other's topics.
