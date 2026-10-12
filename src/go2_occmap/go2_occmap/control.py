"""Velocity freshness and ideal-scan stop gate, shared by simulator and tests."""
import math
import numpy as np


class Watchdog:
    def __init__(self, timeout=.5):
        self.timeout, self.received, self.command = timeout, -math.inf, np.zeros(3)

    def update(self, command, now):
        command = np.asarray(command, dtype=float)
        self.command = command.copy() if command.shape == (3,) and np.isfinite(command).all() else np.zeros(3)
        self.received = now

    def value(self, now):
        return self.command.copy() if 0 <= now-self.received <= self.timeout else np.zeros(3)


def scan_gate(command, ranges):
    command = np.asarray(command, dtype=float).copy()
    if len(ranges) == 0:
        return np.zeros(3)
    angles = np.linspace(-math.pi, math.pi, len(ranges), endpoint=False)
    if command[0] > 0 and np.min(np.asarray(ranges)[abs(angles) < .55]) < .7:
        command[0] = 0.
    return command
