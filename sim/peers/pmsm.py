# Rotor state shared by foc_plant.py (which integrates it) and mt6835_peer.py (which reads the
# shaft angle): a module of its own so both peer scripts import the same instance.
P = 7


class Motor:
    def __init__(self):
        self.id = self.iq = 0.0
        self.wm = 0.0
        self.theta_m = 0.6 / P  # rotor starts 0.6 electrical rad off the d-axis alignment


MOTOR = Motor()
