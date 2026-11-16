# Surface PMSM on the Pioneer joint board's 3-PWM bridge, as their SimpleFOC loop drives it
# (src/main.cpp: BLDCDriver3PWM on PB6/PB7/PB9 = TIM4 CH1/CH2/CH4, 12 V supply). Ported from
# an earlier TIM1-driven plant, with the duty read from TIM4 instead of TIM1, and no shunt
# outputs: their firmware has no current sensing. The rotor angle it integrates is shared with
# mt6835_peer.py through pmsm.MOTOR.
#
# Model (rotor dq frame, amplitude-invariant Park): phase-to-neutral volts from the three duty
# cycles CCRx / ARR; L did/dt = vd - R id + we L iq, L diq/dt = vq - R iq - we L id - we psi;
# J dwm/dt = 1.5 p psi iq - B wm; we = p wm.
#
# MODELED SIMPLIFICATION: averaged switching; the bridge is always enabled (their driver has no
# enable pin set, and MOT_EN PB1 is only written low before driver init); rigid joint; the plant
# steps every 40 us (the firmware's 25 kHz PWM period) with 10 sub-steps.
import math

from pmsm import MOTOR

R, L, PSI, P = 0.2, 1e-4, 0.004, 7
J, B = 2e-4, 1e-5
VBUS = 12.0
TIM4 = 0x40000800
ARR, CCR1, CCR2, CCR4 = 0x2C, 0x34, 0x38, 0x40
STEP_US, SUB = 40, 10
SAMPLE_EVERY = 250  # 10 ms


class Peripheral:
    def __init__(self, ctx):
        self.ctx = ctx
        self.n = 0
        ctx.schedule_periodic(STEP_US)

    def on_access(self, req):
        if req.is_init:
            self.ctx.schedule_periodic(STEP_US)
        req.value = 0

    def on_timer(self):
        rd = lambda off: self.ctx.bus_read(TIM4 + off, 4)
        arr = rd(ARR) or 1
        duty = [min(rd(off) / arr, 1.0) for off in (CCR1, CCR2, CCR4)]
        m = MOTOR
        dt = STEP_US * 1e-6 / SUB
        mean = sum(duty) / 3
        vph = [VBUS * (d - mean) for d in duty]
        for _ in range(SUB):
            th = P * m.theta_m
            c = [math.cos(th - k * 2 * math.pi / 3) for k in range(3)]
            s = [math.sin(th - k * 2 * math.pi / 3) for k in range(3)]
            we = P * m.wm
            vd = 2 / 3 * sum(v * ck for v, ck in zip(vph, c))
            vq = -2 / 3 * sum(v * sk for v, sk in zip(vph, s))
            did = (vd - R * m.id + we * L * m.iq) / L
            diq = (vq - R * m.iq - we * L * m.id - we * PSI) / L
            m.id += did * dt
            m.iq += diq * dt
            m.wm += (1.5 * P * PSI * m.iq - B * m.wm) / J * dt
            m.theta_m += m.wm * dt
        self.n += 1
        if self.n % SAMPLE_EVERY == 0:  # sim/ addition: log the plant state for sim/plot.py
            self.ctx.info(f"PLANT t={self.n * STEP_US / 1e6:.4f} wm={m.wm:.3f} iq={m.iq:.3f} id={m.id:.3f} duty={duty[0]:.3f},{duty[1]:.3f},{duty[2]:.3f}")
