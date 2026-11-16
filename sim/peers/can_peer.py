# CAN master for the Pioneer joint, speaking the protocol their src/main.cpp implements (not
# src/interfacing/dbc/humanoid.dbc, which the joint firmware does not use): standard ID 0x123,
# classic frames. The joint sends "67890" on 0x123 every 100 ms of its own millis() and polls RX
# FIFO0 (filter: 0x123 only) in the same loop; RxData[0..1] are two ASCII digits NN and the
# firmware sets target_velocity = 25 * NN / 100 rad/s (main.cpp: max_target_velocity = 25).
#
# What the firmware does with that target: motor.controller = velocity_openloop. There is no
# speed feedback. The rotor's electrical angle is advanced at target_velocity from micros() at a
# fixed voltage_limit (3 V), so the shaft follows the field as a synchronous machine would.
#
# The speed checked here is the shaft's encoder slope in rad per virtual second (pmsm.MOTOR.theta_m,
# the angle the MT6835 peer reads). The joint's clock comes from its board's HSE (sim/joint-hse4p8.board
# sets 4.8 MHz to match HSE_VALUE); nothing here rescales time.
from pmsm import MOTOR

SAMPLE_US = 10_000  # 10 ms
SLOPE_N = 10        # slope over the last 100 ms of samples
# (virtual time us, ASCII payload, commanded rad/s) — the first setpoint is the boot default (5 rad/s)
CMDS = [(2_500_000, b"32", 8.0), (3_500_000, b"16", 4.0)]
REPORT_US = 5_000_000
SETTLE_BAND = 0.10
SETTLE_HOLD_US = 300_000
BOOT_TARGET = 5.0


class Node:
    def __init__(self, ctx):
        self.ctx = ctx
        self.status = 0
        self.frames = []
        self.samples = []  # (t_us, theta_m rad)
        self.next_cmd = 0
        self.done = False
        ctx.schedule_oneshot(SAMPLE_US)

    def speeds(self):
        out = []
        for i in range(SLOPE_N, len(self.samples)):
            t0, a0 = self.samples[i - SLOPE_N]
            t1, a1 = self.samples[i]
            out.append((t1, (a1 - a0) / ((t1 - t0) * 1e-6)))
        return out

    def report(self, target, start, end, label):
        speeds = [(t, v) for t, v in self.speeds() if start <= t <= end]
        if not speeds:
            return
        tail = [v for t, v in speeds if t >= end - 500_000]
        mean = sum(tail) / len(tail)
        settled = None
        for t, _ in speeds:
            win = [v for tt, v in speeds if t <= tt <= t + SETTLE_HOLD_US]
            if win and t + SETTLE_HOLD_US <= end and all(abs(v - target) <= SETTLE_BAND * target for v in win):
                settled = t
                break
        ok = abs(mean - target) <= SETTLE_BAND * target
        when = f"settled in {(settled - start) / 1e6:.2f} s" if settled is not None else "did NOT settle"
        self.ctx.info(f"{label} {target:.2f} rad/s -> measures {mean:.2f} rad/s virtual ({when})")
        self.results.append(ok and settled is not None)

    def on_timer(self):
        t = self.ctx.now_us
        self.samples.append((t, MOTOR.theta_m))
        if self.next_cmd < len(CMDS) and t >= CMDS[self.next_cmd][0]:
            _, payload, _ = CMDS[self.next_cmd]
            self.ctx.send(0x123, payload + bytes(6))
            self.ctx.info(f"sent target '{payload.decode()}' on 0x123 at {t / 1e6:.2f} s")
            self.next_cmd += 1
        if t >= REPORT_US and not self.done:
            self.done = True
            self.finish()
            return
        if not self.done:
            self.ctx.schedule_oneshot(SAMPLE_US)

    def finish(self):
        self.results = []
        self.report(BOOT_TARGET, 2_000_000, 2_500_000, "boot default  ")
        self.report(8.0, 2_500_000, 3_500_000, "commanded")
        self.report(4.0, 3_500_000, REPORT_US, "commanded")
        if len(self.frames) >= 2:
            mean_iv = (self.frames[-1] - self.frames[0]) / (len(self.frames) - 1)
            self.ctx.info(f"status frames: {len(self.frames)}, mean interval {mean_iv / 1000:.2f} ms virtual")
        if all(self.results):
            self.ctx.info("joint tracked the CAN command: all setpoints within 10% (virtual time)")
        else:
            self.ctx.info("joint did NOT track the CAN command")

    def on_frame(self, frame):
        if frame['id'] == 0x123 and bytes(frame['data'][:5]) == b"67890":
            self.status += 1
            self.frames.append(self.ctx.now_us)
