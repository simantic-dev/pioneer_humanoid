# Rotor state shared by foc_plant.py (which integrates it) and mt6835_peer.py (which reads the
# shaft angle): a module of its own so both peer scripts import the same instance.
from collections import deque

P = 7
HIST = deque(maxlen=2048)  # sensed joint angle per plant tick (arm_plant.py), newest last; the encoder peer delays through it
READS = [0]  # cumulative encoder angle reads by the firmware (mt6835_peer.py)


class Motor:
    def __init__(self):
        self.id = self.iq = 0.0
        self.wm = 0.0
        self.theta_m = 0.6 / P  # rotor starts 0.6 electrical rad off the d-axis alignment


MOTOR = Motor()
