"""Closed-loop wheel speed control and the realistic simulator drivetrain."""
import importlib.util
import math
from pathlib import Path
import random
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from rclpy.time import Time

from svea_core.controllers.speed import SpeedController
from svea_core.models.drivetrain import WheelEncoderSim, esc_speed


CONTROL_DT = 0.1     # twist_consumer loop period [s]
PLANT_DT = 0.025     # sim_svea step [s]
ENCODER_DT = 0.05    # 20 Hz encoder topic
ESC_TAU = 0.1        # Bicycle4DWithESC time constant [s]


def drive(controller, target, seconds, deadband=0.15, gain=0.8, noise=0.0, seed=1):
    """Run controller and simulated car together; return (times, speeds, commands)."""
    rng = random.Random(seed)
    encoder = WheelEncoderSim()
    speed = command = 0.0
    times, speeds, commands = [], [], []
    steps = round(seconds / PLANT_DT)
    for step in range(steps):
        now = step * PLANT_DT
        if step % round(CONTROL_DT / PLANT_DT) == 0:
            command = controller.update(target(now) if callable(target) else target, now)
        wanted = esc_speed(command + rng.gauss(0, noise), deadband, gain)
        speed += PLANT_DT / ESC_TAU * (wanted - speed)
        encoder.advance(speed, PLANT_DT)
        if step % round(ENCODER_DT / PLANT_DT) == 0:
            controller.update_measurement(encoder.read_speed(ENCODER_DT), now)
        times.append(now)
        speeds.append(speed)
        commands.append(command)
    return times, speeds, commands


def mean_after(times, values, start):
    selected = [value for time, value in zip(times, values) if time >= start]
    return sum(selected) / len(selected)


class DrivetrainSimTest(unittest.TestCase):
    def test_esc_deadband_and_gain(self):
        self.assertEqual(esc_speed(0.1, 0.15, 0.8), 0.0)
        self.assertEqual(esc_speed(-0.1, 0.15, 0.8), 0.0)
        self.assertAlmostEqual(esc_speed(0.4, 0.15, 0.8), 0.2)
        self.assertAlmostEqual(esc_speed(-0.4, 0.15, 0.8), -0.2)

    def test_encoder_reports_whole_ticks(self):
        encoder = WheelEncoderSim(wheel_radius=0.055, ticks_per_rev=80)
        self.assertAlmostEqual(encoder.tick_length, 2 * math.pi * 0.055 / 80)
        encoder.advance(0.1, 0.05)                 # 5 mm: one tick
        self.assertAlmostEqual(encoder.read_speed(0.05), encoder.tick_length / 0.05)
        encoder.advance(0.01, 0.05)                # 0.5 mm: no new tick yet
        self.assertEqual(encoder.read_speed(0.05), 0.0)
        encoder.advance(-1.0, 0.05)                # backwards
        self.assertLess(encoder.read_speed(0.05), 0.0)

    def test_open_loop_stalls_or_misses_target(self):
        """Without speed control the realistic car does not hold 0.1 m/s."""
        speed = 0.0
        for _ in range(200):
            speed += PLANT_DT / ESC_TAU * (esc_speed(0.1, 0.15, 0.8) - speed)
        self.assertEqual(speed, 0.0)


class SpeedControllerTest(unittest.TestCase):
    def test_holds_slow_speed_through_deadband(self):
        controller = SpeedController()
        times, speeds, commands = drive(controller, 0.1, 8.0)
        self.assertAlmostEqual(mean_after(times, speeds, 4.0), 0.1, delta=0.01)
        self.assertLess(max(speeds), 0.15, 'no lurch past 1.5x the target')
        self.assertLessEqual(max(commands), controller.max_command)
        self.assertEqual(controller.status, 'tracking')

    def test_holds_speed_with_command_noise(self):
        controller = SpeedController()
        times, speeds, _ = drive(controller, 0.1, 8.0, noise=0.02, seed=7)
        self.assertAlmostEqual(mean_after(times, speeds, 4.0), 0.1, delta=0.015)

    def test_min_command_speeds_up_start(self):
        slow = drive(SpeedController(), 0.1, 3.0)
        fast = drive(SpeedController(min_command=0.15), 0.1, 3.0)
        def first_moving(run):
            return next(time for time, speed in zip(run[0], run[1]) if speed > 0.05)
        self.assertLess(first_moving(fast), first_moving(slow))

    def test_reference_ramps_up(self):
        controller = SpeedController(accel_limit=0.3)
        controller.update_measurement(0.0, 0.0)
        controller.update(0.2, 0.0)
        controller.update_measurement(0.0, 0.1)
        controller.update(0.2, 0.1)
        self.assertAlmostEqual(controller.reference, 0.03)

    def test_stop_target_is_immediate(self):
        controller = SpeedController()
        drive(controller, 0.1, 4.0)
        self.assertEqual(controller.update(0.0, 4.1), 0.0)
        self.assertEqual(controller.status, 'stopped')
        self.assertEqual(controller.reference, 0.0)
        self.assertEqual(controller._integral, 0.0)

    def test_stops_without_feedback(self):
        controller = SpeedController(feedback_timeout=0.3)
        self.assertEqual(controller.update(0.1, 0.0), 0.0)
        self.assertEqual(controller.status, 'no feedback')
        controller.update_measurement(0.0, 1.0)
        controller.update(0.1, 1.1)
        self.assertEqual(controller.status, 'tracking')
        self.assertEqual(controller.update(0.1, 1.5), 0.0)
        self.assertEqual(controller.status, 'no feedback')

    def test_stall_latches_until_stop(self):
        """Dead encoder: output is capped, then the controller stops and latches."""
        controller = SpeedController(max_command=0.4, stall_timeout=2.0)
        now, outputs = 0.0, []
        while now < 6.0:
            controller.update_measurement(0.0, now)
            outputs.append(controller.update(0.1, now))
            now += CONTROL_DT
        self.assertLessEqual(max(outputs), 0.4)
        self.assertEqual(outputs[-1], 0.0)
        self.assertIsNotNone(controller.fault)
        controller.update_measurement(0.0, now)
        self.assertEqual(controller.update(0.1, now), 0.0, 'fault stays latched')
        controller.update(0.0, now + 0.1)
        self.assertIsNone(controller.fault)

    def test_reverse_waits_for_forward_motion_to_end(self):
        controller = SpeedController()
        controller.update_measurement(0.1, 0.0)
        controller.update(0.1, 0.0)
        controller.update_measurement(0.1, 0.1)
        self.assertEqual(controller.update(-0.1, 0.1), 0.0)
        self.assertEqual(controller.status, 'waiting for stop')

    def test_reverse_tracks(self):
        controller = SpeedController()
        times, speeds, commands = drive(controller, -0.1, 8.0)
        self.assertAlmostEqual(mean_after(times, speeds, 4.0), -0.1, delta=0.01)
        self.assertGreaterEqual(min(commands), -controller.max_command)

    def test_rejects_bad_settings(self):
        with self.assertRaises(ValueError):
            SpeedController(kp=-1)
        with self.assertRaises(ValueError):
            SpeedController(max_command=0)
        with self.assertRaises(ValueError):
            SpeedController(min_command=0.5, max_command=0.4)


class TwistConsumerSpeedControlTest(unittest.TestCase):
    def load(self):
        path = Path(__file__).parents[2] / 'svea_examples/scripts/twist_consumer.py'
        spec = importlib.util.spec_from_file_location('twist_speed', path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module

    def consumer(self, controller):
        now = Time(nanoseconds=1_000_000_000)
        return SimpleNamespace(get_clock=lambda: SimpleNamespace(now=lambda: now),
            get_logger=lambda: Mock(), actuation=Mock(), require_px4_armed=False,
            _pump_until=None, _last_msg=now, cmd_timeout=.6, _velocity=0.1, _steering=0.,
            steering_sign=1., velocity_sign=1., _speed_controller=controller,
            _speed_status=None, speed_feedback_topic='mavros/wheel_odometry/velocity',
            _log_speed_status=lambda: None)

    def test_off_sends_the_open_loop_command(self):
        module = self.load()
        consumer = self.consumer(None)
        with patch.object(module.time, 'monotonic', return_value=1.):
            module.twist_consumer.loop.callback(consumer)
        consumer.actuation.send_control.assert_called_with(0., 0.1)

    def test_on_sends_the_controller_command(self):
        module = self.load()
        controller = Mock()
        controller.update.return_value = 0.23
        consumer = self.consumer(controller)
        with patch.object(module.time, 'monotonic', return_value=1.):
            module.twist_consumer.loop.callback(consumer)
        controller.update.assert_called_with(0.1, 1.)
        consumer.actuation.send_control.assert_called_with(0., 0.23)

    def test_stale_drive_command_reaches_controller_as_stop(self):
        module = self.load()
        controller = Mock()
        controller.update.return_value = 0.
        consumer = self.consumer(controller)
        consumer._last_msg = Time(nanoseconds=0)
        with patch.object(module.time, 'monotonic', return_value=1.):
            module.twist_consumer.loop.callback(consumer)
        controller.update.assert_called_with(0., 1.)


if __name__ == '__main__':
    unittest.main()
