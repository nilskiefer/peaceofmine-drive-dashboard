"""Wheel faults, PI saturation and motor-command watchdogs without hardware."""
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from svea_core.speed_control import DEFAULTS, SpeedController, WheelFeedback, validate


class SpeedControlTest(unittest.TestCase):
    def feedback(self,controller,now,left=.2,right=.4):
        controller.feedback.distance(1_000_000_000+round(now*1e9),[left*now,right*now],now)

    def test_higher_wheel_is_used_and_disagreement_is_reported(self):
        feedback=WheelFeedback()
        feedback.distance(1_000_000_000,[0.,0.],0.)
        feedback.distance(1_200_000_000,[0.,.08],.2)
        state=feedback.snapshot(.2)
        self.assertEqual(state['selected'],'right')
        self.assertEqual(state['health'],'disagreement')
        self.assertAlmostEqual(state['measured_mps'],.4)
        feedback.distance(1_200_000_000,[0.,.08],.6)
        self.assertFalse(feedback.snapshot(.8)['fresh'],'Replaying a packet must not renew encoder feedback')

    def test_one_invalid_wheel_does_not_hide_the_other_and_spikes_are_rejected(self):
        feedback=WheelFeedback()
        feedback.distance(1_000_000_000,[0.,float('nan')],0.)
        feedback.distance(1_200_000_000,[.04,float('nan')],.2)
        state=feedback.snapshot(.2)
        self.assertTrue(state['fresh'])
        self.assertEqual(state['selected'],'left')
        self.assertEqual(state['health'],'disagreement')
        feedback.distance(1_300_000_000,[1000.,1000.],.3)
        self.assertFalse(feedback.snapshot(.3)['fresh'])

    def test_open_loop_needs_no_encoder_and_closed_loop_fault_requires_neutral(self):
        controller=SpeedController()
        state=controller.step(.4,0.)
        self.assertGreater(state['throttle_percent'],0.)
        self.assertEqual(state['encoder']['health'],'missing')
        controller.configure(dict(DEFAULTS,mode='closed_loop'))
        self.assertEqual(controller.step(.4,.1)['state'],'neutral_required')
        controller.step(0.,.2)
        self.assertEqual(controller.step(.4,.3)['state'],'encoder_fault')
        self.feedback(controller,.4);self.feedback(controller,.6)
        self.assertEqual(controller.step(.4,.6)['throttle_percent'],0.,'Feedback recovery must not restart a held command')
        controller.step(0.,.65)
        self.assertGreater(controller.step(.4,.7)['throttle_percent'],0.)

    def test_pi_removes_persistent_speed_error_and_resets_at_stop(self):
        results=[]
        for integral in (0.,5.):
            controller=SpeedController(dict(DEFAULTS,mode='closed_loop',ki=integral))
            speed=distance=0.
            for i in range(1001):
                now=i*.05
                distance+=speed*.05
                controller.feedback.distance(1_000_000_000+round(now*1e9),[distance,distance],now)
                state=controller.step(.4 if i>1 else 0.,now)
                speed+=(state['throttle_percent']*.01-speed)*.2
            results.append(abs(speed-.4))
            controller.step(0.,50.1)
            self.assertEqual(controller.integral,0.)
        self.assertLess(results[1],.025)
        self.assertGreater(results[0],results[1]*3)

    def test_no_wheel_movement_fault_and_anti_windup(self):
        controller=SpeedController(dict(DEFAULTS,mode='closed_loop',max_output_percent=20.,kp=1000.,ki=100.))
        for i in range(110):
            now=i*.1
            controller.feedback.distance(1_000_000_000+round(now*1e9),[0.,0.],now)
            state=controller.step(.8 if i>1 else 0.,now)
            self.assertLessEqual(state['throttle_percent'],20.)
            self.assertEqual(controller.integral,0.)
            if now<10.2:
                self.assertNotEqual(state['state'],'encoder_fault')
        self.assertEqual(state['state'],'encoder_fault')
        self.assertEqual(state['encoder']['health'],'no_motion')
        self.assertEqual(state['throttle_percent'],0.)

    def test_settings_reject_nonfinite_values_and_unsafe_limits(self):
        for key,value in (('kp',float('nan')),('ki',-1.),('max_output_percent',100.),('max_speed_mps',0.),('mode','automatic')):
            with self.subTest(key=key),self.assertRaises(ValueError):validate(dict(DEFAULTS,**{key:value}))


class ConsumerSpeedTest(unittest.TestCase):
    def setUp(self):
        path=Path(__file__).parents[2]/'svea_examples/scripts/twist_consumer.py'
        spec=importlib.util.spec_from_file_location('speed_consumer_test',path)
        self.module=importlib.util.module_from_spec(spec);spec.loader.exec_module(self.module)

    def test_saturated_integrator_cannot_bypass_timeout_and_steering_stays_independent(self):
        controller=SpeedController(dict(DEFAULTS,mode='closed_loop'))
        controller.feedback.distance(1_000_000_000,[0.,0.],.8)
        controller.feedback.distance(1_200_000_000,[.02,.02],1.)
        controller.integral=40.
        consumer=SimpleNamespace(speed_controller=controller,speed_settings_error=None,
            _speed_state_at=0.,speed_state=Mock(),actuation=Mock(max_speed=1.7),
            require_px4_armed=False,_pump_until=None,_command_received=.99,cmd_timeout=.25,
            _velocity=.4,_steering=.2,steering_sign=1.,velocity_sign=1.)
        with patch.object(self.module.time,'monotonic',return_value=1.):self.module.twist_consumer.loop.callback(consumer)
        self.assertEqual(consumer.actuation.send_control.call_args.args[0],.2)
        self.assertGreater(consumer.actuation.send_control.call_args.args[1],0.)
        with patch.object(self.module.time,'monotonic',return_value=1.3):self.module.twist_consumer.loop.callback(consumer)
        consumer.actuation.send_control.assert_called_with(0.,0.)
        self.assertEqual(controller.integral,0.)

    def test_repeated_settings_publications_do_not_pulse_motor_to_zero(self):
        consumer=SimpleNamespace(speed_controller=SpeedController(),speed_settings_error=None,actuation=Mock())
        message=SimpleNamespace(data=json.dumps(dict(drive_speed=DEFAULTS)))
        self.module.twist_consumer.settings_cb.callback(consumer,message)
        consumer.actuation.send_control.assert_not_called()

    def test_runtime_speed_limit_changes_scale_inputs_and_stop_existing_commands(self):
        import threading
        from svea_core import speed_control
        path=Path(__file__).parents[1]/'scripts/operator_control_node.py'
        spec=importlib.util.spec_from_file_location('speed_operator_test',path)
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        owner=SimpleNamespace(_lock=threading.RLock(),_max_velocity=.8,_keyboard_limits=(1.,1.),
            _drive_input='rc',_apply_drive_locked=Mock(),_emit_cmd_vel=Mock())
        message=SimpleNamespace(data=json.dumps(dict(driving=dict(input='rc',forward_percent=100,reverse_percent=100),
            drive_speed=dict(DEFAULTS,max_speed_mps=.3))))
        module.OperatorControl._drive_settings_cb(owner,message)
        self.assertEqual(owner._max_velocity,.3)
        owner._apply_drive_locked.assert_called_once_with(0.,0.,False)
        owner._apply_drive_locked.reset_mock()
        module.OperatorControl._drive_settings_cb(owner,message)
        owner._apply_drive_locked.assert_not_called()

    def test_invalid_settings_recovery_cannot_restart_a_held_command(self):
        controller=SpeedController()
        consumer=SimpleNamespace(speed_controller=controller,speed_settings_error=None,actuation=Mock())
        invalid=SimpleNamespace(data=json.dumps(dict(drive_speed=DEFAULTS,error='Settings document unreadable')))
        self.module.twist_consumer.settings_cb.callback(consumer,invalid)
        self.assertIsNotNone(consumer.speed_settings_error)
        valid=SimpleNamespace(data=json.dumps(dict(drive_speed=DEFAULTS)))
        self.module.twist_consumer.settings_cb.callback(consumer,valid)
        self.assertIsNone(consumer.speed_settings_error)
        self.assertEqual(controller.step(.4,1.)['state'],'neutral_required')
        controller.step(0.,1.1)
        self.assertGreater(controller.step(.4,1.2)['throttle_percent'],0.)

    def test_ros_pipeline_stops_on_encoder_loss_and_open_loop_can_drive_without_it(self):
        import tempfile
        import time
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.parameter import Parameter
        from geometry_msgs.msg import Twist
        from mavros_msgs.msg import WheelOdomStamped, ManualControl
        from std_msgs.msg import String
        rclpy.init(args=['--ros-args','-r','__ns:=/speed_control_test'])
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'settings.json'
            path.write_text(json.dumps(dict(drive_speed=dict(DEFAULTS,mode='closed_loop'))))
            node=self.module.twist_consumer(parameter_overrides=[Parameter('config_file',value=str(path))])
            observer=rclpy.create_node('speed_observer')
            executor=SingleThreadedExecutor()
            commands=[];telemetry=[]
            observer.create_subscription(ManualControl,'mavros/manual_control/send',commands.append,10)
            observer.create_subscription(String,'drive/speed_state',lambda msg:telemetry.append(json.loads(msg.data)),10)
            encoders=observer.create_publisher(WheelOdomStamped,'mavros/wheel_odometry/distance',10)
            drive=observer.create_publisher(Twist,'cmd_vel',10)
            settings=observer.create_publisher(String,'operator/settings',10)
            distance=[0.,0.];last_encoder=[0.]
            def spin(seconds,target=0.,feedback=True):
                end=time.monotonic()+seconds
                while time.monotonic()<end:
                    now=time.monotonic()
                    message=Twist();message.linear.x=target;message.angular.z=.1 if target else 0.
                    drive.publish(message)
                    if feedback and now-last_encoder[0]>=.09:
                        distance[1]+=.02
                        packet=WheelOdomStamped(data=distance)
                        packet.header.stamp=node.get_clock().now().to_msg()
                        encoders.publish(packet);last_encoder[0]=now
                    executor.spin_once(timeout_sec=.02)
            try:
                node.__rosonic_startup__(node)
                executor.add_node(node);executor.add_node(observer)
                spin(.6)
                spin(.25,.4)
                self.assertTrue(telemetry[-1]['encoder']['fresh'])
                self.assertEqual(telemetry[-1]['encoder']['selected'],'right')
                self.assertLess(commands[-1].z,500.,'PI must reach the motor output')
                spin(.8,.4,False)
                self.assertEqual(telemetry[-1]['state'],'encoder_fault')
                self.assertEqual(commands[-1].z,500.,'Encoder loss must neutralize a held drive command')
                settings.publish(String(data=json.dumps(dict(drive_speed=dict(DEFAULTS,max_speed_mps=.3)))))
                spin(.2,0.,False)
                spin(.2,.4,False)
                self.assertEqual(telemetry[-1]['mode'],'open_loop')
                self.assertAlmostEqual(telemetry[-1]['target_mps'],.3)
                self.assertLess(commands[-1].z,500.,'Explicit open loop must not depend on encoder data')
                spin(.1,0.,False)
            finally:
                node.__rosonic_shutdown__(node)
                executor.shutdown()
                observer.destroy_node();node.destroy_node()
                rclpy.try_shutdown()
