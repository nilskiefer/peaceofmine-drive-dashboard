"""Closed-loop wheel speed control for the SVEA drive motor.

The low-level interface only accepts an open-loop speed command, which the ESC
turns into throttle. Near standstill that mapping is poor: small commands fall
inside the ESC deadband and the car does not move, and slightly larger ones make
it lurch. This controller closes the loop on the measured wheel speed.

Every speed here is in m/s. The output is the speed command passed to
``ActuationInterface.send_control``, which converts it to throttle.

The class has no ROS dependencies so it can be tested on its own.
"""

import math


class SpeedController:
    """Feedforward plus PI wheel speed controller with safety limits.

    Args:
        kp: Proportional gain, command m/s per m/s of speed error.
        ki: Integral gain, command m/s per (m/s * s) of speed error.
        kff: Feedforward gain from target speed to command.
        min_command: Command added whenever the car should move, to get
            through the ESC deadband. Measure it on the car.
        max_command: Largest command magnitude the controller may send.
        accel_limit: Largest rate at which the speed reference rises [m/s^2].
            Slowing down and stopping are not rate limited.
        filter_tau: Time constant of the low-pass filter on measured speed [s].
        feedback_timeout: Stop if no speed measurement arrived for this long [s].
        stall_timeout: Stop and latch a fault if the command stays at
            max_command for this long while the wheels do not turn [s].
        stall_speed: Measured speed below which the wheels count as stopped [m/s].
    """

    STOP_TARGET = 1e-3       # [m/s] targets below this mean "stop"
    OPPOSITE_SPEED = 0.03    # [m/s] still rolling the other way: wait at neutral
    MAX_DT = 0.5             # [s] ignore longer gaps between updates

    def __init__(self, kp=0.6, ki=1.2, kff=1.0, min_command=0.0, max_command=0.4,
                 accel_limit=0.3, filter_tau=0.3, feedback_timeout=0.3,
                 stall_timeout=2.0, stall_speed=0.02):
        values = dict(kp=kp, ki=ki, kff=kff, min_command=min_command,
                      max_command=max_command, accel_limit=accel_limit,
                      filter_tau=filter_tau, feedback_timeout=feedback_timeout,
                      stall_timeout=stall_timeout, stall_speed=stall_speed)
        for name, value in values.items():
            if not math.isfinite(value) or value < 0:
                raise ValueError(f'{name} must be finite and non-negative, not {value}')
        if max_command <= 0:
            raise ValueError('max_command must be positive')
        if min_command > max_command:
            raise ValueError('min_command must not exceed max_command')

        self.kp, self.ki, self.kff = kp, ki, kff
        self.min_command, self.max_command = min_command, max_command
        self.accel_limit = accel_limit
        self.filter_tau = filter_tau
        self.feedback_timeout = feedback_timeout
        self.stall_timeout, self.stall_speed = stall_timeout, stall_speed

        self.measured = 0.0       # filtered measured speed [m/s]
        self.fault = None         # latched fault text, cleared by a stop target
        self.status = 'stopped'   # what the controller did on the last update
        self._measured_at = None
        self._last_update = None
        self.reset()

    def reset(self):
        """Forget the reference, integrator and stall timer."""
        self.reference = 0.0
        self._integral = 0.0
        self._stall_time = 0.0

    def update_measurement(self, speed, now):
        """Add a wheel speed measurement [m/s] taken at time ``now`` [s]."""
        speed = float(speed)
        if not math.isfinite(speed):
            return
        if self._measured_at is None or self.filter_tau == 0:
            self.measured = speed
        else:
            dt = min(max(now - self._measured_at, 0.0), self.MAX_DT)
            alpha = 1.0 - math.exp(-dt / self.filter_tau)
            self.measured += alpha * (speed - self.measured)
        self._measured_at = now

    def update(self, target, now):
        """Return the speed command for ``target`` [m/s] at time ``now`` [s]."""
        dt = 0.0 if self._last_update is None else now - self._last_update
        dt = min(max(dt, 0.0), self.MAX_DT)
        self._last_update = now

        target = float(target)
        if not math.isfinite(target) or abs(target) < self.STOP_TARGET:
            self.fault = None
            return self._neutral('stopped')
        if self.fault is not None:
            return self._neutral('fault')
        if self._measured_at is None or now - self._measured_at > self.feedback_timeout:
            return self._neutral('no feedback')

        # Reverse only after passing through zero, then ramp the reference up.
        if self.reference * target < 0:
            self.reset()
        if abs(target) > abs(self.reference):
            step = min(abs(target) - abs(self.reference), self.accel_limit * dt)
            self.reference = math.copysign(abs(self.reference) + step, target)
        else:
            self.reference = target

        direction = math.copysign(1.0, target)
        reference = abs(self.reference)
        speed = direction * self.measured    # speed along the wanted direction
        if speed < -self.OPPOSITE_SPEED:
            self._integral = 0.0
            self._stall_time = 0.0
            self.status = 'waiting for stop'
            return 0.0

        error = reference - speed
        feedforward = self.min_command + self.kff * reference if reference > 0 else 0.0
        unclamped = feedforward + self.kp * error + self._integral
        # Integrate only while the output is not pushing further into a limit.
        if (0.0 < unclamped < self.max_command
                or (unclamped >= self.max_command and error < 0)
                or (unclamped <= 0.0 and error > 0)):
            self._integral += self.ki * error * dt
            self._integral = min(max(self._integral, -self.max_command), self.max_command)
        command = feedforward + self.kp * error + self._integral
        command = min(max(command, 0.0), self.max_command)

        if command >= self.max_command and abs(self.measured) < self.stall_speed:
            self._stall_time += dt
            if self._stall_time > self.stall_timeout:
                self.fault = (f'wheels did not turn for {self.stall_timeout:.1f} s '
                              f'at the maximum command')
                return self._neutral('fault')
        else:
            self._stall_time = 0.0

        self.status = 'tracking'
        return direction * command

    def _neutral(self, status):
        self.reset()
        self.status = status
        return 0.0
