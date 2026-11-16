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
# Time base: the joint's millis()/micros() run about 5x slower than wall/virtual time in this
# simulation (the firmware's HSE_VALUE is 4.8 MHz; the scenario's crystal is 24 MHz, see README
# "Clock"), so the firmware's "rad/s" are rad per firmware second. The status frames carry the
# firmware's own clock: one per 100 ms firmware time. This peer measures that scale from the
# frame spacing (k = mean frame interval / 100 ms) and converts the shaft's virtual-time slope
# (pmsm.MOTOR.theta_m, the angle the MT6835 peer reads) to firmware rad/s: speed_fw = k * speed_virtual.
#
# shortcut: the frame-spacing scale assumes the firmware's clock is steady over the run; it is
# not a firmware measurement. Upgrade: read the firmware's millis() through a debug probe.
from pmsm import MOTOR

SAMPLE_US = 10_000  # 10 ms
SLOPE_N = 10        # slope over the last 100 ms of samples
PERIOD_FW_US = 100_000
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
        self.samples = []  # (t_us, theta_m rad, unwrapped)
        self.next_cmd = 0
        self.done = False
        ctx.schedule_oneshot(SAMPLE_US)

    def fw_speeds(self):
        k = self.scale()
        out = []
        for i in range(SLOPE_N, len(self.samples)):
            t0, a0 = self.samples[i - SLOPE_N]
            t1, a1 = self.samples[i]
            v = (a1 - a0) / ((t1 - t0) * 1e-6)
            out.append((t1, k * v))
        return out

    def scale(self):
        if len(self.frames) < 2:
            return 1.0
        mean_iv = (self.frames[-1] - self.frames[0]) / (len(self.frames) - 1)
        return mean_iv / PERIOD_FW_US

    def report(self, target, start, end, label):
        speeds = [(t, f) for t, f in self.fw_speeds() if start <= t <= end]
        if not speeds:
            return
        tail = [f for t, f in speeds if t >= end - 500_000]
        mean = sum(tail) / len(tail)
        settled = None
        for t, _ in speeds:
            win = [f for tt, f in speeds if t <= tt <= t + SETTLE_HOLD_US]
            if win and t + SETTLE_HOLD_US <= end and all(abs(f - target) <= SETTLE_BAND * target for f in win):
                settled = t
                break
        ok = abs(mean - target) <= SETTLE_BAND * target
        when = f"settled in {(settled - start) / 1e6:.2f} s" if settled is not None else "did NOT settle"
        self.ctx.info(f"{label} {target:.2f} rad/s -> firmware measures {mean:.2f} rad/s ({when})")
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
        k = self.scale()
        self.ctx.info(f"firmware clock: {k:.3f} firmware s per virtual s, {len(self.frames)} status frames")
        if all(self.results):
            self.ctx.info("joint tracked the CAN command: all setpoints within 10% (firmware time)")
        else:
            self.ctx.info("joint did NOT track the CAN command")

    def on_frame(self, frame):
        if frame['id'] == 0x123 and bytes(frame['data'][:5]) == b"67890":
            self.status += 1
            self.frames.append(self.ctx.now_us)
