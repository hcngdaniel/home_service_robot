#!/usr/bin/env python3


class VelocitySmoother:
    def __init__(self, max_velocity, max_acceleration):
        self.max_velocity = abs(max_velocity)
        self.max_acceleration = abs(max_acceleration)
        self.last_velocity = 0

    def __call__(self, velocity):
        velocity = min(velocity, self.last_velocity + self.max_acceleration)
        velocity = max(velocity, self.last_velocity - self.max_acceleration)
        velocity = min(velocity, self.max_velocity)
        velocity = max(velocity, -self.max_velocity)
        self.last_velocity = velocity
        return velocity

