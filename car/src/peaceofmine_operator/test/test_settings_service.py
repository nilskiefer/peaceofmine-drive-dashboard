import json
import tempfile
from pathlib import Path
import unittest
import rclpy
from rclpy.parameter import Parameter
from std_msgs.msg import String
from peaceofmine_interfaces.srv import SaveSettings
from peaceofmine_operator import configuration
from test_arm_node import module


class SettingsServiceTest(unittest.TestCase):
    def test_drive_speed_mode_and_pi_gains_survive_restart(self):
        from svea_core.speed_control import DEFAULTS
        rclpy.init()
        with tempfile.TemporaryDirectory() as tmp:
            path=str(Path(tmp)/'operator_config.json')
            params=[Parameter('config_file',value=path)]
            node_type=module('operator_settings_node').OperatorSettings
            node=node_type(parameter_overrides=params)
            try:
                value=dict(DEFAULTS,mode='closed_loop',kp=18.,ki=2.,max_speed_mps=.6,max_output_percent=35.)
                node.command(String(data=json.dumps(dict(request_id='speed',section='drive_speed',value=value))))
                self.assertEqual(configuration.load(path)['drive_speed'],value)
                invalid=SaveSettings.Request(section='drive_speed',value_json=json.dumps(dict(value,ki=-1.)))
                self.assertFalse(node.save(invalid,SaveSettings.Response()).success)
                self.assertEqual(configuration.load(path)['drive_speed'],value)
                node.destroy_node()
                node=node_type(parameter_overrides=params)
                self.assertEqual(node.shared['drive_speed'],value)
            finally:
                node.destroy_node()
                rclpy.shutdown()

    def test_sweep_range_is_saved_separately_and_cannot_exceed_physical_arm(self):
        rclpy.init()
        with tempfile.TemporaryDirectory() as tmp:
            path=str(Path(tmp)/'operator_config.json')
            physical=dict(servo_id=1,minimum=1024,center=2048,maximum=3072)
            configuration.save_section(path,'arm',physical)
            params=[Parameter('config_file',value=path)]
            node_type=module('operator_settings_node').OperatorSettings
            node=node_type(parameter_overrides=params)
            try:
                valid=dict(minimum_deg=-22.5,maximum_deg=45.)
                request=dict(request_id='sweep-range',section='arm_sweep',value=valid)
                node.command(String(data=json.dumps(request)))
                self.assertEqual(configuration.load(path)['arm_sweep'],valid)
                self.assertEqual(configuration.load(path)['arm'],physical)
                bad=SaveSettings.Request(section='arm_sweep',value_json=json.dumps(dict(minimum_deg=-100.,maximum_deg=45.)))
                response=node.save(bad,SaveSettings.Response())
                self.assertFalse(response.success)
                self.assertEqual(configuration.load(path)['arm_sweep'],valid)
                node.destroy_node()
                node=node_type(parameter_overrides=params)
                self.assertEqual(node.shared['arm_sweep'],valid)
            finally:
                node.destroy_node()
                rclpy.shutdown()

    def test_guidance_requires_measurements_and_survives_settings_restart(self):
        rclpy.init()
        with tempfile.TemporaryDirectory() as tmp:
            path = str(Path(tmp) / 'operator_config.json')
            options = [Parameter('config_file', value=path)]
            node_type = module('operator_settings_node').OperatorSettings
            node = node_type(parameter_overrides=options)
            try:
                self.assertIsNone(node.shared['probe_guidance'])
                valid = dict(pivot_x_m=.2, pivot_y_m=-.1, coil_radius_m=.5, probe_radius_m=.6)
                request = dict(request_id='geometry', section='probe_guidance', value=valid)
                node.command(String(data=json.dumps(request)))
                self.assertEqual(configuration.load(path)['probe_guidance'], valid)
                node.command(String(data=json.dumps(dict(request, value=dict(valid, coil_radius_m=0)))))
                self.assertEqual(configuration.load(path)['probe_guidance'], valid)
                node.destroy_node()
                node = node_type(parameter_overrides=options)
                self.assertEqual(node.shared['probe_guidance'], valid)
            finally:
                node.destroy_node()
                rclpy.shutdown()

    def test_dashboard_drive_settings_survive_node_restart(self):
        rclpy.init()
        with tempfile.TemporaryDirectory() as tmp:
            path = str(Path(tmp) / 'operator_config.json')
            options = [Parameter('config_file', value=path)]
            node_type = module('operator_settings_node').OperatorSettings
            node = node_type(parameter_overrides=options)
            try:
                valid = dict(input='rc', forward_percent=45, reverse_percent=0)
                request = dict(request_id='drive', section='driving', value=valid)
                node.command(String(data=json.dumps(request)))
                self.assertEqual(configuration.load(path)['driving'], valid)
                for invalid in [dict(valid, input='unknown'), dict(valid, reverse_percent=-1),
                                dict(valid, forward_percent=True), dict(valid, forward_percent=101)]:
                    node.command(String(data=json.dumps(dict(request, value=invalid))))
                    self.assertEqual(configuration.load(path)['driving'], valid)
                node.destroy_node()
                node = node_type(parameter_overrides=options)
                self.assertEqual(node.shared['driving'], valid)
            finally:
                node.destroy_node()
                rclpy.shutdown()

    def test_contact_source_and_force_limit_survive_restart(self):
        from peaceofmine_operator.probe_contact_settings import DEFAULTS
        rclpy.init()
        with tempfile.TemporaryDirectory() as tmp:
            path = str(Path(tmp) / 'operator_config.json')
            options = [Parameter('config_file', value=path)]
            node_type = module('operator_settings_node').OperatorSettings
            node = node_type(parameter_overrides=options)
            try:
                self.assertEqual(node.shared['probe_contact']['source'], 'adc')
                good = {**DEFAULTS, 'source':'esp32', 'threshold_counts':50., 'zero_counts':None}
                request = dict(request_id='contact', section='probe_contact', value=good)
                node.command(String(data=json.dumps(request)))
                self.assertEqual(configuration.load(path)['probe_contact'], good)
                good['zero_counts'] = 1000.
                response = node.save(SaveSettings.Request(section='probe_contact',value_json=json.dumps(good),
                    expected_revision=configuration.load(path)['revision']),SaveSettings.Response())
                self.assertTrue(response.success,response.message)
                node.command(String(data=json.dumps({**request,'value':{**good,'zero_counts':None}})))
                self.assertEqual(configuration.load(path)['probe_contact']['zero_counts'],1000.)
                node.command(String(data=json.dumps({**request, 'value':{**good, 'threshold_counts':-1}})))
                self.assertEqual(configuration.load(path)['probe_contact'], good)
                node.destroy_node()
                node = node_type(parameter_overrides=options)
                self.assertEqual(node.shared['probe_contact'], good)
            finally:
                node.destroy_node()
                rclpy.shutdown()

    def test_probe_stroke_speeds_and_home_load_survive_restart(self):
        rclpy.init()
        with tempfile.TemporaryDirectory() as tmp:
            path = str(Path(tmp) / 'operator_config.json')
            options = [Parameter('config_file', value=path)]
            node_type = module('operator_settings_node').OperatorSettings
            node = node_type(parameter_overrides=options)
            try:
                valid = dict(down_mm_s=1.75, retract_mm_s=13., retract_slow_mm_s=3., home_load_percent=15., contact_slowdown_factor=3.)
                request = dict(request_id='stroke', section='probe_stroke', value=valid)
                node.command(String(data=json.dumps(request)))
                self.assertEqual(configuration.load(path)['probe_stroke'], valid)
                for invalid in (dict(valid, down_mm_s=0.), dict(valid, retract_mm_s=-1.), dict(valid, home_load_percent=61.)):
                    node.command(String(data=json.dumps(dict(request, value=invalid))))
                    self.assertEqual(configuration.load(path)['probe_stroke'], valid)
                node.destroy_node()
                node = node_type(parameter_overrides=options)
                self.assertEqual(node.shared['probe_stroke'], valid)
            finally:
                node.destroy_node()
                rclpy.shutdown()

    def test_detector_accepts_fractional_calibration_span(self):
        rclpy.init()
        with tempfile.TemporaryDirectory() as tmp:
            path=str(Path(tmp)/'operator_config.json')
            node=module('operator_settings_node').OperatorSettings(parameter_overrides=[Parameter('config_file',value=path)])
            try:
                request=SaveSettings.Request(section='metal_detector',value_json=json.dumps(dict(
                    baseline_adc=125.7,full_response_adc=126.2,reference_voltage=5.)),expected_revision=0)
                result=node.save(request,SaveSettings.Response())
                self.assertTrue(result.success,result.message)
                self.assertEqual(configuration.load(path)['metal_detector']['full_response_adc'],126.2)
                request.value_json=json.dumps(dict(baseline_adc=125.7,full_response_adc=125.7,reference_voltage=5.))
                request.expected_revision=result.revision
                self.assertFalse(node.save(request,SaveSettings.Response()).success)
            finally:
                node.destroy_node()
                rclpy.shutdown()

    def test_revision_conflicts_and_invalid_documents_preserve_saved_settings(self):
        rclpy.init()
        with tempfile.TemporaryDirectory() as tmp:
            path=str(Path(tmp)/'operator_config.json')
            node=module('operator_settings_node').OperatorSettings(parameter_overrides=[Parameter('config_file',value=path)])
            try:
                request=SaveSettings.Request(section='arm_motion',value_json=json.dumps(dict(max_speed_deg_s=10.,acceleration_deg_s2=34.332)),expected_revision=0)
                result=node.save(request,SaveSettings.Response())
                self.assertTrue(result.success,result.message)
                self.assertEqual(result.revision,1)
                original=Path(path).read_text()
                self.assertFalse(node.save(request,SaveSettings.Response()).success)
                for value in ('[]','null','{"max_speed_deg_s":0}','{broken'):
                    request.value_json=value
                    request.expected_revision=1
                    self.assertFalse(node.save(request,SaveSettings.Response()).success)
                    self.assertEqual(Path(path).read_text(),original)
                self.assertEqual(configuration.load(path)['section_revisions']['arm_motion'],1)
            finally:
                node.destroy_node()
                rclpy.shutdown()
