


class PID:
    def __init__(self, kp, ki, kd, output_limit=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit

        self.integral = 0.0
        self.last_error = 0.0
        self.first = True

    def update(self, target, measured, dt):
        error = target - measured

        if self.first:
            derivative = 0.0
            self.first = False
        else:
            derivative = (error - self.last_error) / dt if dt > 0 else 0.0

        self.integral += error * dt
        self.last_error = error

        u = self.kp*error + self.ki*self.integral + self.kd*derivative

        if self.output_limit is not None:
            u = max(-self.output_limit, min(self.output_limit, u))

        return u
