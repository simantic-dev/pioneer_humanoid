# The cases swept by run_cases.py. Each one runs the same firmware binary (sim/build/joint-arm.elf) against a different arm / sensor.
# `src` says where a number comes from: "datasheet", "catalogue", or "chosen" (my choice, not a measurement).
# Every case sets ENC_OFFSET_RAD = pi (encoder reads pi at the hang pose, away from its wrap) unless it says otherwise.
import math
DEG = math.pi / 180
AS = dict(ENC_BITS=14, ENC_NOISE_RAD=0.068 * DEG, ENC_DELAY_US=110)  # AS5047P: 14-bit, 0.068 deg rms (1 sigma), ~110 us SPI propagation delay (datasheet)
CASES = [
    dict(name="ideal", label="Ideal", params="exact encoder, no friction, no backlash, 0.1 kg", arm={}, env={}),
    dict(name="as5047p", label="AS5047P-class encoder", params="14-bit, noise 0.068 deg rms, delay 110 us", arm={}, env=dict(AS), src="datasheet (AS5047P, ams-OSRAM)"),
    dict(name="as5047p_wrap", label="Same encoder, zero at rest pose", params="as the previous panel, but the encoder's 0/2pi wrap sits at the arm's hang pose", arm={}, env=dict(AS, ENC_OFFSET_RAD=0), scenario="sim/scenario-arm-zero0.yaml", src="setup, not a part (firmware built with ZERO0=1)"),
    dict(name="rough_sensor", label="Rough encoder", params="12-bit, noise 0.3 deg rms, delay 5 ms", arm={}, env=dict(ENC_BITS=12, ENC_NOISE_RAD=0.3 * DEG, ENC_DELAY_US=5000), src="chosen"),
    dict(name="lag_20ms", label="Encoder lag", params="delay 20 ms, otherwise exact", arm={}, env=dict(ENC_DELAY_US=20000), src="chosen"),
    dict(name="friction", label="Joint friction", params="Coulomb 0.03 N*m (5% of the stand-in's 0.63 peak) + viscous 0.01", arm=dict(frictionloss=0.03), env={}, src="chosen; Coulomb + viscous is MuJoCo's own model"),
    dict(name="payload_0p2", label="Payload 0.2 kg", params="tip mass 0.2 kg (gravity torque 0.59 N*m)", arm=dict(payload=0.2), env={}, src="chosen"),
    dict(name="lash_15arcmin", label="Backlash 15 arcmin", params="gap 0.25 deg, encoder on the motor, arm hangs (gravity preloads the gap)", arm=dict(backlash=15 / 60 * DEG), env={}, src="catalogue: standard planetary 5-15 arcmin"),
    dict(name="combined", label="Combined", params="AS5047P-class encoder + friction + 15 arcmin gap + 0.15 kg", arm=dict(frictionloss=0.03, backlash=15 / 60 * DEG, payload=0.15), env=dict(AS), src="mix of the above"),
    dict(name="flat_ideal", label="Horizontal plane, ideal", params="no gravity on the joint (drawn from the side); exact encoder, no gap", arm=dict(gravity=False), env={}),
    dict(name="flat_lash_motor", label="Horizontal, 1 deg gap, encoder on motor", params="no gravity, gap 1 deg, encoder on the motor", arm=dict(gravity=False, backlash=1 * DEG), env={}, src="chosen"),
    dict(name="flat_lash_load", label="Horizontal, 1 deg gap, encoder on load", params="no gravity, gap 1 deg, encoder on the load (the gap is inside the loop)", arm=dict(gravity=False, backlash=1 * DEG), env=dict(ARM_ENC="load"), src="chosen"),
]
