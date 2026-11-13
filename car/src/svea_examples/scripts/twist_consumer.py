#! /usr/bin/env python3

import math
import time
import json
from pathlib import Path

from svea_core import rosonic as rx
from svea_core.interfaces import ActuationInterface
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, String
from mavros_msgs.msg import State, WheelOdomStamped
from rclpy.qos import qos_profile_sensor_data
from svea_core.speed_control import SpeedController, DEFAULTS


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
    config_file = rx.Parameter('')
    encoder_topic = rx.Parameter('mavros/wheel_odometry/distance')
    speed_state = rx.Publisher(String,'drive/speed_state')

    actuation = ActuationInterface()

    _velocity = 0.0
    _steering = 0.0
    _last_msg = None
    _pump_until = None
    _armed_until = 0.
    _state_stamp = 0

    def on_startup(self):
        if self.twist_type not in ("geometry_msgs/msg/Twist",
                                   "geometry_msgs/msg/TwistStamped"):
            raise TypeError(f'Invalid message type for {self.twist_top}: {self.twist_type}')

        if not math.isfinite(self.min_speed_for_steering) or self.min_speed_for_steering <= 0:
            raise ValueError('min_speed_for_steering must be finite and positive')

        if self.use_difflock:
            self.actuation.enable_difflock()

        self._last_msg = self.get_clock().now()
        self._command_received = time.monotonic()
        legacy=dict(DEFAULTS,max_speed_mps=min(1.7,self.max_velocity),max_output_percent=self.actuation.MAX_SPEED_PERCENT)
        self.speed_controller = SpeedController(legacy)
        self.speed_settings_error = None
        self._speed_state_at = 0.
        if self.config_file:
            try:
                document=json.loads(Path(self.config_file).read_text())
                if not isinstance(document,dict) or document.get('schema_version',1)!=1:
                    raise ValueError('Unsupported operator settings document')
                self.speed_controller=SpeedController(document.get('drive_speed',DEFAULTS))
            except (OSError,ValueError,TypeError,AttributeError) as exc:
                self.speed_settings_error=str(exc)
                self.speed_controller.neutral_required=True
        self.get_logger().info(
            f'twist_consumer listening on "{self.twist_top}" ({self.twist_type})')

    def _omega_to_steering(self, v, w):
        """Ackermann steering with a speed floor so steering also works at rest."""
        speed = max(abs(v), self.min_speed_for_steering)
        delta = math.atan(self.wheelbase * w / speed)
        return max(-self.max_steering, min(self.max_steering, delta))

    def on_shutdown(self):
        self.actuation.send_xtr(xtr1=0.0)
        self.actuation.send_control(0.0, 0.0)
        self.actuation.loop()  # Publish neutral before ROS publishers are destroyed.

    @rx.Subscriber(String,'operator/settings')
    def settings_cb(self,msg):
        changed=False
        try:
            value=json.loads(msg.data)
            if not isinstance(value,dict):raise ValueError('Invalid speed settings message')
            if value.get('error'):raise ValueError(value['error'])
            if 'drive_speed' not in value:return
            changed=value['drive_speed'] != self.speed_controller.options
            self.speed_controller.configure(value['drive_speed'])
            self.speed_settings_error=None
        except (ValueError,TypeError) as exc:
            self.speed_settings_error=str(exc)
            self.speed_controller.neutral_required=True
        # A settings change or failure must never leave the old output latched.
        if changed or self.speed_settings_error:
            self.actuation.send_control(0.,0.)

    @rx.Subscriber(WheelOdomStamped,encoder_topic,qos_profile_sensor_data)
    def encoder_cb(self,msg):
        stamp=msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
        age=(self.get_clock().now().nanoseconds-stamp)/1e9
        self.speed_controller.feedback.distance(stamp,msg.data,time.monotonic(),age)

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
        if not math.isfinite(v) or not math.isfinite(w):
            v=w=0.
        limit=self.speed_controller.options['max_speed_mps'] if self.config_file else self.max_velocity
        v = max(-limit, min(limit, v))

        self._velocity = v
        self._steering = self._omega_to_steering(v, w)
        self._last_msg = self.get_clock().now()
        self._command_received = time.monotonic()

    @rx.Timer(0.05)
    def loop(self):
        now = time.monotonic()
        allowed = not self.require_px4_armed or now < self._armed_until
        if self.require_px4_armed and now >= self._armed_until:
            self._velocity = self._steering = 0.
            self._pump_until = 0.
        if self._pump_until is not None and now >= self._pump_until:
            self._pump_until = None
            self.actuation.send_xtr(xtr1=0.0)
        age = now-self._command_received if hasattr(self,'_command_received') else (self.get_clock().now() - self._last_msg).nanoseconds * 1e-9
        if age > self.cmd_timeout:
            self._velocity = 0.0
            self._steering = 0.0

        velocity=self._velocity
        controller=getattr(self,'speed_controller',None)
        if controller is not None:
            fault=getattr(self,'speed_settings_error',None)
            reason='settings_error' if fault else 'disarmed' if not allowed else 'command_timeout' if age>self.cmd_timeout else ''
            state=controller.step(velocity,now,enabled=allowed and age<=self.cmd_timeout and not fault,
                reason=reason,speed_scale=self.actuation.max_speed)
            if fault:state['reason']='Speed settings unavailable: '+fault
            velocity=state['throttle_percent']*self.actuation.max_speed/100.
            if now-self._speed_state_at>=.1:
                self._speed_state_at=now
                self.speed_state.publish(String(data=json.dumps(state,allow_nan=False)))
        self.actuation.send_control(self._steering * self.steering_sign,velocity * self.velocity_sign)


if __name__ == '__main__':
    twist_consumer.main()
