"""Device selection and GNSS wiring, without starting physical drivers."""
import importlib.util
from collections import defaultdict
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from launch import LaunchContext
from launch.actions import ExecuteProcess
from launch.utilities import perform_substitutions
import rclpy


ROOT=Path(__file__).parents[2]


def module(name,path):
    spec=importlib.util.spec_from_file_location(name,path)
    result=importlib.util.module_from_spec(spec);spec.loader.exec_module(result)
    return result


class PeripheralLaunchTest(unittest.TestCase):
    def test_operator_forwards_same_gnss_identity_to_rtk(self):
        launch=module('operator_vehicle',ROOT/'peaceofmine_operator/launch/operator_vehicle.launch.py')
        context=LaunchContext()
        context.extend_locals({'ros_specific_arguments':defaultdict(str)})
        context.launch_configurations.update(launch.DEFAULTS)
        context.launch_configurations.update(use_localization='true',gnss_device='/dev/serial/by-id/my-gnss',gnss_baud='38400')
        process=next(action for action in launch.bringup(context) if isinstance(action,ExecuteProcess))
        args=[perform_substitutions(context,part) for part in process.cmd]
        self.assertIn('rtk_device:=/dev/serial/by-id/my-gnss',args)
        self.assertIn('rtk_baud:=38400',args)

    def test_rtk_resolves_usb_link_and_honors_baud_override(self):
        rtk=module('rtk_manager',ROOT/'svea_localization/scripts/rtk_manager.py')
        with tempfile.TemporaryDirectory() as tmp:
            link=Path(tmp)/'usb-gnss';link.symlink_to('/dev/ttyACM99')
            rclpy.init(args=['--ros-args','-p',f'device:={link}','-p','baud:=38400'])
            try:
                with patch.object(rtk,'Serial') as serial,patch.object(rtk,'UBXReader'), \
                        patch.object(rtk.RTKManager,'_init_pub'),patch.object(rtk.RTKManager,'_init_sub'), \
                        patch.object(rtk.RTKManager,'setup_receiver'),patch.object(rtk.RTKManager,'start_serial_read'):
                    node=rtk.RTKManager()
                    try:
                        self.assertEqual((node.rateUART1,node.rateUSB),(0,1))
                        self.assertEqual(serial.call_args.args[:2],(str(link),38400))
                    finally:node.destroy_node()
            finally:rclpy.shutdown()
