#!/usr/bin/env python3


class PID:
    def __init__(self, p, i, d):
        self.kp = p
        self.ki = i
        self.kd = d
        self.error_sum = 0
        self.last_error = 0

    def reset(self):
        self.error_sum = 0
        self.last_error = 0

    def __call__(self, target, current):
        ret = 0
        error = target - current
        self.error_sum += error
        ret += self.kp * error
        ret += self.ki * self.error_sum
        ret += self.kd * (error - self.last_error)
        self.last_error = error
        return ret
