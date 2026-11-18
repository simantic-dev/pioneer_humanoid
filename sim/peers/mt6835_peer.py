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

from pmsm import MOTOR

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
                a = int(MOTOR.theta_m % (2 * math.pi) / (2 * math.pi) * CPR) % CPR
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
