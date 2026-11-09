"""Simple drivetrain effects for a more realistic simulated SVEA.

The default simulator turns a speed command into exactly that speed. A real car
has an ESC deadband, a throttle-to-speed gain that is not exactly known, and
wheel encoders with a coarse resolution. These helpers add those effects so a
speed controller can be tried in simulation first.

The module has no ROS dependencies.
"""

import math


def esc_speed(command, deadband, gain):
    """Speed [m/s] the motor reaches for a speed ``command`` [m/s].

    Commands smaller than ``deadband`` [m/s] do not move the car. Above it the
    speed rises with ``gain`` from zero.
    """
    magnitude = abs(command) - deadband
    if magnitude <= 0:
        return 0.0
    return math.copysign(gain * magnitude, command)


class WheelEncoderSim:
    """Quantized wheel encoder that reports speed from whole ticks.

    Args:
        wheel_radius: Wheel radius [m].
        ticks_per_rev: Encoder counts per wheel revolution.
    """

    def __init__(self, wheel_radius=0.055, ticks_per_rev=80):
        if wheel_radius <= 0 or ticks_per_rev <= 0:
            raise ValueError('wheel_radius and ticks_per_rev must be positive')
        self.tick_length = 2 * math.pi * wheel_radius / ticks_per_rev  # [m]
        self.distance = 0.0      # true signed distance travelled [m]
        self._reported_ticks = 0

    @property
    def ticks(self):
        """Whole ticks counted so far."""
        return math.floor(self.distance / self.tick_length)

    def advance(self, speed, dt):
        """Move the wheel at ``speed`` [m/s] for ``dt`` [s]."""
        self.distance += speed * dt

    def read_speed(self, period):
        """Speed [m/s] from the ticks counted since the last read ``period`` [s] ago."""
        ticks = self.ticks
        delta = ticks - self._reported_ticks
        self._reported_ticks = ticks
        return delta * self.tick_length / period if period > 0 else 0.0
