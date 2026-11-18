# MagnTek MT6835 21-bit magnetic angle encoder on SPI1 (PA5/PA6/PA7, CS PA0, mode 3) reading the
# simulated PMSM's shaft (pmsm.MOTOR). Host type: `spi-device`.
#
# Only the transaction their firmware sends is modelled: SimpleFOCDrivers' MT6835::readRawAngle21
# clocks 6 bytes [0xA0, 0x03, 0, 0, 0, 0] (opcode 0b1010 = burst-read angle, register 0x003) and
# takes back registers 0x003..0x006: ANGLE[20:13], ANGLE[12:5], ANGLE[4:0]<<3 | STATUS[2:0],
# CRC8 (poly 0x07, init 0, over the three angle bytes; MT6835 datasheet sec. 7.4, same as
# SimpleFOCDrivers' calcCrc). Status is 0 (no overspeed / weak-field / under-voltage). Any other
# command reads back zeros.
import math
import os
import random

from pmsm import HIST, MOTOR, READS

# Sensor imperfections for sim/mujoco/run_cases.py, from the environment; all off by default (the ideal MT6835 above).
# ENC_DELAY_US: the angle the firmware reads is the one from that long ago (a multiple of the plant's 40 us tick).
# ENC_NOISE_RAD: Gaussian noise, 1 sigma. ENC_BITS: quantise the angle to that many bits before it is packed into the 21-bit field.
DELAY_TICKS = round(float(os.environ.get("ENC_DELAY_US", "0")) / 40)
NOISE = float(os.environ.get("ENC_NOISE_RAD", "0"))
BITS = int(os.environ.get("ENC_BITS", "21"))
# ENC_OFFSET_RAD: where the encoder reads at angle 0 (the magnet / zero placement). 0 puts its 0/2pi wrap at the arm's hang pose.
OFFSET = float(os.environ.get("ENC_OFFSET_RAD", "0"))
RNG = random.Random(int(os.environ.get("ENC_SEED", "1")))

CPR = 1 << 21


def crc8(data):
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


class Slave:
    def __init__(self, ctx):
        self.ctx = ctx
        self.beat = 0
        self.cmd = []
        self.out = b""
        self.reads = 0

    def on_transmit(self, b):
        i = self.beat
        self.beat += 1
        if i < 2:
            self.cmd.append(b)
            if i == 1 and self.cmd == [0xA0, 0x03]:
                ang = HIST[-1 - DELAY_TICKS] if DELAY_TICKS and len(HIST) > DELAY_TICKS else MOTOR.theta_m
                ang += OFFSET
                if NOISE:
                    ang += RNG.gauss(0.0, NOISE)
                if BITS < 21:
                    q = 2 * math.pi / (1 << BITS)
                    ang = round(ang / q) * q
                a = int(ang % (2 * math.pi) / (2 * math.pi) * CPR) % CPR
                READS[0] += 1
                regs = bytes([a >> 13, (a >> 5) & 0xFF, (a & 0x1F) << 3])
                self.out = regs + bytes([crc8(regs)])
                self.reads += 1
                if self.reads in (1, 1000, 10000):
                    self.ctx.info(f"MT6835 angle read #{self.reads}: raw {a}")
            return 0x00
        return self.out[i - 2] if i - 2 < len(self.out) else 0x00

    def on_finish(self):
        self.beat = 0
        self.cmd = []
        self.out = b""
