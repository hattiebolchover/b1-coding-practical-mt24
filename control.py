# control.py
class PDController:
    """
    Simple Proportional-Derivative (PD) controller
    u[t] = Kp*e[t] + Kd*(e[t] - e[t-1])
    """

    def __init__(self, kp=0.15, kd=0.6):
        self.kp = kp
        self.kd = kd
        self.prev_error = 0.0

    def compute(self, reference, output):
        error = reference - output
        control = self.kp * error + self.kd * (error - self.prev_error)
        self.prev_error = error
        return control

