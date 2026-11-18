# MuJoCo arm on the Pioneer joint board's 3-PWM bridge (TIM4 CH1/CH2/CH4), in lockstep with the
# firmware: every STEP_US of virtual time this peer reads the PWM duty the firmware has written,
# integrates the motor's dq currents, hands the resulting shaft torque to MuJoCo as the hinge
# actuator control, steps MuJoCo by the same STEP_US, and publishes the hinge angle and speed
# through pmsm.MOTOR, which mt6835_peer.py reads as the encoder shaft angle.
# The mechanics (link, tip mass, gravity, joint damping, rotor armature) are MuJoCo's, from
# sim/mujoco/arm.xml. The electrical side is the motor equations of foc_plant.py.
# MODELED SIMPLIFICATION: R, L, PSI, P are stand-ins (the repo does not name the motor); averaged
# switching; the bridge is always enabled; torque and speed are exchanged once per STEP_US.
import math
import os

import mujoco

from pmsm import MOTOR

R, L, PSI, P = 0.2, 1e-4, 0.004, 7
VBUS = 12.0
TIM4 = 0x40000800
ARR, CCR1, CCR2, CCR4 = 0x2C, 0x34, 0x38, 0x40
STEP_US, SUB = 40, 10
SAMPLE_EVERY = 250  # 10 ms
MODEL = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "mujoco", "arm.xml")


class Peripheral:
    def __init__(self, ctx):
        self.ctx = ctx
        self.n = 0
        self.id = self.iq = 0.0
        self.m = mujoco.MjModel.from_xml_path(MODEL)
        self.d = mujoco.MjData(self.m)
        assert abs(self.m.opt.timestep - STEP_US * 1e-6) < 1e-12, "arm.xml timestep must equal STEP_US"
        MOTOR.theta_m, MOTOR.wm = float(self.d.qpos[0]), 0.0
        ctx.info(f"ARM MuJoCo {mujoco.__version__}, {os.path.basename(MODEL)}, step {STEP_US} us")
        ctx.schedule_periodic(STEP_US)

    def on_access(self, req):
        if req.is_init:
            self.ctx.schedule_periodic(STEP_US)
        req.value = 0

    def on_timer(self):
        rd = lambda off: self.ctx.bus_read(TIM4 + off, 4)
        arr = rd(ARR) or 1
        duty = [min(rd(off) / arr, 1.0) for off in (CCR1, CCR2, CCR4)]
        mean = sum(duty) / 3
        vph = [VBUS * (d - mean) for d in duty]
        dt = STEP_US * 1e-6 / SUB
        wm, th_m, torque = float(self.d.qvel[0]), float(self.d.qpos[0]), 0.0
        for _ in range(SUB):
            th = P * th_m
            c = [math.cos(th - k * 2 * math.pi / 3) for k in range(3)]
            s = [math.sin(th - k * 2 * math.pi / 3) for k in range(3)]
            we = P * wm
            vd = 2 / 3 * sum(v * ck for v, ck in zip(vph, c))
            vq = -2 / 3 * sum(v * sk for v, sk in zip(vph, s))
            self.id += (vd - R * self.id + we * L * self.iq) / L * dt
            self.iq += (vq - R * self.iq - we * L * self.id - we * PSI) / L * dt
            torque += 1.5 * P * PSI * self.iq / SUB
            th_m += wm * dt
        self.d.ctrl[0] = torque
        mujoco.mj_step(self.m, self.d)
        MOTOR.theta_m, MOTOR.wm = float(self.d.qpos[0]), float(self.d.qvel[0])
        self.n += 1
        if self.n % SAMPLE_EVERY == 0:
            self.ctx.info(f"ARM t={self.n * STEP_US / 1e6:.4f} q={MOTOR.theta_m:.4f} qd={MOTOR.wm:.3f} tau={torque:.4f} iq={self.iq:.3f}")
