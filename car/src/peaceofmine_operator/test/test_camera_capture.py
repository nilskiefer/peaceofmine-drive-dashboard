"""Camera startup consumes shared settings, without opening a physical camera."""
import copy
import importlib.util
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

try:
    import pyudev
    from ament_index_python.packages import get_package_prefix
except ImportError:
    pyudev = None


@unittest.skipIf(pyudev is None, 'ROS camera environment required')
class CameraCaptureTest(unittest.TestCase):
    def module(self):
        spec=importlib.util.spec_from_file_location('camera_auto',Path(__file__).parents[1]/'scripts/usb_camera_auto.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        return module

    def test_persistent_names_survive_camera_renumbering(self):
        module=self.module()
        with tempfile.TemporaryDirectory() as tmp:
            directory=Path(tmp)
            links=[directory/'camera-a-video-index0',directory/'camera-b-video-index0']
            for link,target in zip(links,('video9','video0')):link.symlink_to(directory/target)
            with patch.object(module,'glob',return_value=[str(link) for link in reversed(links)]):
                self.assertEqual(module.matching_capture_devices(object()),[str(link) for link in links])
                links[0].unlink();links[0].symlink_to(directory/'video1')
                self.assertEqual(module.matching_capture_devices(object()),[str(link) for link in links])

    def test_explicit_camera_link_is_passed_to_driver(self):
        module=self.module()
        with tempfile.TemporaryDirectory() as tmp:
            device=Path(tmp)/'video4';device.touch()
            link=Path(tmp)/'camera-by-id';link.symlink_to(device)
            with patch.dict(os.environ,{'POM_CAMERA_DEVICE':str(link)}):
                self.assertEqual(module.resolve_device(),str(link))

    def test_camera_without_serial_uses_physical_path(self):
        module=self.module()
        path='/dev/v4l/by-path/platform-usb-0:1.2-video-index0'
        with patch.object(module,'glob',side_effect=lambda pattern:[] if '/by-id/' in pattern else [path]):
            self.assertEqual(module.matching_capture_devices(object()),[path])

    def test_capture_settings_reach_camera_driver(self):
        from peaceofmine_operator.camera_settings import DEFAULTS
        spec=importlib.util.spec_from_file_location('camera_auto',Path(__file__).parents[1]/'scripts/usb_camera_auto.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'operator_config.json'
            cameras=copy.deepcopy(DEFAULTS)
            cameras['capture']['forward'].update(device='/dev/v4l/by-id/camera',width=800,framerate=20)
            path.write_text(json.dumps({'cameras':cameras}))
            with patch.dict(os.environ,{'POM_CONFIG_FILE':str(path),'POM_CAMERA_INDEX':'0'}), \
                    patch.object(module,'resolve_device',return_value='/dev/video4'), \
                    patch.object(module,'get_package_prefix',return_value='/opt/ros/jazzy'), \
                    patch.object(module.os,'execv') as execute:
                module.main()
                args=execute.call_args.args[1]
                self.assertIn('image_width:=800',args)
                self.assertIn('framerate:=20.0',args)
                self.assertIn('video_device:=/dev/video4',args)
                self.assertEqual(os.environ['POM_CAMERA_DEVICE'],'/dev/v4l/by-id/camera')
