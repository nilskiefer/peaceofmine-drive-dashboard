#! /usr/bin/env python3

import math
import time

from rclpy.qos import qos_profile_sensor_data
from svea_core import rosonic as rx
from svea_core.controllers.speed import SpeedController
from svea_core.interfaces import ActuationInterface
from geometry_msgs.msg import Twist, TwistWithCovarianceStamped
from std_msgs.msg import Float32
from mavros_msgs.msg import State


class twist_consumer(rx.Node):
    """Twist (v, omega) -> SVEA actuation (steering angle, velocity)."""

    twist_top  = rx.Parameter('cmd_vel')
    twist_type = rx.Parameter('geometry_msgs/msg/Twist')

    wheelbase    = rx.Parameter(0.324)   # [m]
    max_steering = rx.Parameter(0.524)   # [rad]
    max_velocity = rx.Parameter(0.8)     # [m/s]
    min_speed_for_steering = rx.Parameter(0.05)

    velocity_sign = rx.Parameter(1.0)
    steering_sign = rx.Parameter(1.0)

    cmd_timeout  = rx.Parameter(0.6)     # [s]
    use_difflock = rx.Parameter(True)
    pump_top     = rx.Parameter('actuation/pump_percent')
    require_px4_armed = rx.Parameter(False)

    # Closed-loop speed control on measured wheel speed. Off keeps open loop.
    speed_control = rx.Parameter(False)
    speed_feedback_topic = rx.Parameter('mavros/wheel_odometry/velocity')
    speed_feedback_sign = rx.Parameter(1.0)
    speed_kp = rx.Parameter(0.6)
    speed_ki = rx.Parameter(1.2)
    speed_kff = rx.Parameter(1.0)
    speed_min_command = rx.Parameter(0.0)      # [m/s] ESC deadband offset
    speed_max_command = rx.Parameter(0.4)      # [m/s] output limit
    speed_accel_limit = rx.Parameter(0.3)      # [m/s^2]
    speed_filter_tau = rx.Parameter(0.3)       # [s]
    speed_feedback_timeout = rx.Parameter(0.3) # [s]
    speed_stall_timeout = rx.Parameter(2.0)    # [s]

    actuation = ActuationInterface()

    _velocity = 0.0
    _steering = 0.0
    _last_msg = None
    _pump_until = None
    _armed_until = 0.
    _state_stamp = 0
    _speed_controller = None

    def on_startup(self):
        if self.twist_type not in ("geometry_msgs/msg/Twist",
                                   "geometry_msgs/msg/TwistStamped"):
            raise TypeError(f'Invalid message type for {self.twist_top}: {self.twist_type}')

        if not math.isfinite(self.min_speed_for_steering) or self.min_speed_for_steering <= 0:
            raise ValueError('min_speed_for_steering must be finite and positive')

        if self.use_difflock:
            self.actuation.enable_difflock()

        self._last_msg = self.get_clock().now()
        self.get_logger().info(
            f'twist_consumer listening on "{self.twist_top}" ({self.twist_type})')

        if self.speed_control:
            self._speed_controller = SpeedController(
                kp=self.speed_kp, ki=self.speed_ki, kff=self.speed_kff,
                min_command=self.speed_min_command,
                max_command=self.speed_max_command,
                accel_limit=self.speed_accel_limit,
                filter_tau=self.speed_filter_tau,
                feedback_timeout=self.speed_feedback_timeout,
                stall_timeout=self.speed_stall_timeout)
            self._speed_status = None
            self.create_subscription(TwistWithCovarianceStamped,
                                     self.speed_feedback_topic,
                                     self.speed_feedback_cb,
                                     qos_profile_sensor_data)
            self.get_logger().info(
                f'Closed-loop speed control on "{self.speed_feedback_topic}"')

    def speed_feedback_cb(self, msg):
        speed = self.speed_feedback_sign * msg.twist.twist.linear.x
        self._speed_controller.update_measurement(speed, time.monotonic())

    def _omega_to_steering(self, v, w):
        """Ackermann steering with a speed floor so steering also works at rest."""
        speed = max(abs(v), self.min_speed_for_steering)
        delta = math.atan(self.wheelbase * w / speed)
        return max(-self.max_steering, min(self.max_steering, delta))

    def on_shutdown(self):
        self.actuation.send_xtr(xtr1=0.0)
        self.actuation.send_control(0.0, 0.0)
        self.actuation.loop()  # Publish neutral before ROS publishers are destroyed.

    @rx.Subscriber(State, 'mavros/state')
    def state_cb(self, msg):
        if not self.require_px4_armed:
            return
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        if msg.connected and msg.armed and stamp > self._state_stamp:
            self._armed_until = time.monotonic() + 3.
        elif not msg.connected or not msg.armed or stamp < self._state_stamp:
            self._armed_until = 0.
            self._velocity = self._steering = 0.
            self._pump_until = None
            self.actuation.send_xtr(xtr1=0.)
            self.actuation.send_control(0., 0.)
        self._state_stamp = stamp if msg.connected else 0

    @rx.Subscriber(Float32, pump_top)
    def pump_cb(self, msg: Float32):
        percent = msg.data if math.isfinite(msg.data) and 0 <= msg.data <= 100 else 0.
        if self.require_px4_armed and time.monotonic() >= self._armed_until:
            percent = 0.
        self._pump_until = time.monotonic() + 0.25
        self.actuation.send_xtr(xtr1=percent)

    @rx.Subscriber(twist_type, twist_top)
    def twist_cb(self, msg):
        tw = msg if isinstance(msg, Twist) else msg.twist

        v = float(tw.linear.x)
        w = float(tw.angular.z)
        v = max(-self.max_velocity, min(self.max_velocity, v))

        self._velocity = v
        self._steering = self._omega_to_steering(v, w)
        self._last_msg = self.get_clock().now()

    @rx.Timer(0.1)
    def loop(self):
        now = time.monotonic()
        if self.require_px4_armed and now >= self._armed_until:
            self._velocity = self._steering = 0.
            self._pump_until = 0.
        if self._pump_until is not None and now >= self._pump_until:
            self._pump_until = None
            self.actuation.send_xtr(xtr1=0.0)
        age = (self.get_clock().now() - self._last_msg).nanoseconds * 1e-9
        if age > self.cmd_timeout:
            self._velocity = 0.0
            self._steering = 0.0

        velocity = self._velocity
        if self._speed_controller is not None:
            velocity = self._speed_controller.update(velocity, now)
            self._log_speed_status()

        self.actuation.send_control(self._steering * self.steering_sign,
                                    velocity * self.velocity_sign)

    def _log_speed_status(self):
        controller = self._speed_controller
        status = controller.fault or controller.status
        if status == self._speed_status:
            return
        self._speed_status = status
        if controller.fault:
            self.get_logger().error(f'Speed control stopped the car: {controller.fault}')
        elif status == 'no feedback':
            self.get_logger().warning(
                f'Speed control is waiting for wheel speed on "{self.speed_feedback_topic}"')
        else:
            self.get_logger().info(f'Speed control: {status}')


if __name__ == '__main__':
    twist_consumer.main()
