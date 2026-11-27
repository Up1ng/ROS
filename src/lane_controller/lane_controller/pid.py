"""Time-aware PID with filtered derivative and conditional integration."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class PidConfig:
    kp: float = 2.1
    ki: float = 0.0
    kd: float = 0.0
    max_angular: float = 0.95
    integral_limit: float = 0.5
    derivative_tau: float = 0.12
    max_dt: float = 0.45

    def validate(self):
        values = (self.kp, self.ki, self.kd, self.integral_limit, self.derivative_tau)
        if not all(math.isfinite(v) and v >= 0 for v in values):
            raise ValueError('PID gains and limits must be finite and nonnegative')
        if not all(math.isfinite(v) and v > 0 for v in (self.max_angular, self.max_dt)):
            raise ValueError('max_angular and max_dt must be finite and positive')


def clamp(value, bound):
    return max(-bound, min(bound, value))


class PID:
    def __init__(self, config):
        self.config = config
        self.reset()

    def reset(self):
        self.previous_time = None
        self.previous_error = 0.0
        self.integral = 0.0
        self.derivative = 0.0

    def update(self, error, stamp):
        cfg = self.config
        dt = None if self.previous_time is None else stamp - self.previous_time
        if dt is None or not 0 < dt <= cfg.max_dt:
            self.reset()
        else:
            alpha = dt / (cfg.derivative_tau + dt)
            rate = (error - self.previous_error) / dt
            self.derivative += alpha * (rate - self.derivative)
            candidate = clamp(self.integral + error * dt, cfg.integral_limit)
            trial = cfg.kp * error + cfg.ki * candidate + cfg.kd * self.derivative
            # Integrate only if inside limits, or if the error pulls back from saturation.
            if cfg.ki and (abs(trial) <= cfg.max_angular or trial * error < 0):
                self.integral = candidate
        self.previous_time, self.previous_error = stamp, error
        total = cfg.kp * error + cfg.ki * self.integral + cfg.kd * self.derivative
        return clamp(total, cfg.max_angular)
