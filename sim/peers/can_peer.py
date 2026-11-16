# CAN master for the Pioneer joint, speaking the protocol their src/main.cpp implements (not
# src/interfacing/dbc/humanoid.dbc, which the joint firmware does not use): standard ID 0x123,
# classic frames. The joint sends "67890" on 0x123 every 100 ms (firmware time) and polls RX
# FIFO0 (filter: 0x123 only) at the same rate; RxData[0..1] are two ASCII digits NN and
# target_velocity = 25 * NN / 100 rad/s.
#
# The peer measures the shaft speed from the simulated PMSM (pmsm.MOTOR) before and after it
# commands "32" (8 rad/s, from the boot default of 5 rad/s) and reports the ratio. A ratio,
# because the firmware's millis()/micros() run 5x fast on the NUCLEO's 24 MHz HSE (see README),
# which scales both speeds alike.
from pmsm import MOTOR

CMD_AT_US, MEASURE_US = 3_200_000, 280_000
CMD = b"32"


class Node:
    def __init__(self, ctx):
        self.ctx = ctx
        self.status = 0
        self.before = None
        ctx.schedule_oneshot(CMD_AT_US - 50_000)

    def on_timer(self):
        if self.before is None:
            self.before = MOTOR.wm
            self.ctx.info(f"shaft {self.before:.1f} rad/s before command, {self.status} status frames")
            self.ctx.send(0x123, CMD + bytes(6))
            self.ctx.info("sent target '32' on 0x123")
            self.ctx.schedule_oneshot(MEASURE_US)
            return
        after = MOTOR.wm
        ratio = after / self.before if self.before else 0.0
        self.ctx.info(f"shaft {after:.1f} rad/s after command, ratio {ratio:.2f}, {self.status} status frames")
        self.ctx.info("joint followed CAN command" if 1.45 < ratio < 1.75 else "joint did NOT follow CAN command")

    def on_frame(self, frame):
        if frame['id'] == 0x123 and bytes(frame['data'][:5]) == b"67890":
            self.status += 1
