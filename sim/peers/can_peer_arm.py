# CAN master for the MuJoCo arm build (ARM=1 sim/build.sh): same protocol as can_peer.py (std ID
# 0x123, two ASCII digits NN), but the patched firmware reads NN as a target angle of NN/100 * pi rad.
# Two steps. The peer only sends them; sim/plot-arm.py judges the response from the logged hinge angle.
import math

from pmsm import MOTOR

STEPS = [(6_000_000, b"30"), (8_500_000, b"60")]  # (virtual us, digits)
HOLD_US = 2_300_000


class Node:
    def __init__(self, ctx):
        self.ctx = ctx
        self.status = 0
        self.i = 0
        self.checking = False
        ctx.schedule_oneshot(STEPS[0][0])

    def on_timer(self):
        if not self.checking:
            t, nn = STEPS[self.i]
            self.target = math.pi * int(nn) / 100
            self.ctx.send(0x123, nn + bytes(6))
            self.ctx.info(f"ARM_CMD target {self.target:.4f} rad ('{nn.decode()}'), hinge at {MOTOR.theta_m:.4f} rad")
            self.checking = True
            self.ctx.schedule_oneshot(HOLD_US)
            return
        self.ctx.info(f"ARM_END hold over, {self.status} status frames")
        self.checking = False
        self.i += 1
        if self.i < len(STEPS):
            self.ctx.schedule_oneshot(STEPS[self.i][0] - STEPS[self.i - 1][0] - HOLD_US)

    def on_frame(self, frame):
        if frame['id'] == 0x123 and bytes(frame['data'][:5]) == b"67890":
            self.status += 1
