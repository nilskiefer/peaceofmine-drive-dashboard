#!/usr/bin/env python3
"""Record mission events independently of browser connections and GNSS availability."""
import json
import math
from pathlib import Path
import threading
import time
import uuid

from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix, Joy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TwistWithCovarianceStamped
from mavros_msgs.msg import RCIn, State
from std_msgs.msg import Bool, String, Float32
from peaceofmine_interfaces.msg import AdcSamples, ServoState, ServoCommand, ActuatorCommand
from rosidl_runtime_py.convert import message_to_ordereddict

from peaceofmine_operator import configuration
from peaceofmine_operator.field_gnss import DEFAULTS, FRESH_S, FieldLog, position, validate
from peaceofmine_operator.node_runner import run_node
from peaceofmine_operator.mission_session import MissionSession, clean, software_version


class FieldRecorder(Node):
    def __init__(self):
        super().__init__('field_recorder')
        simulation = self.declare_parameter('simulation', False).value
        self.device = self.declare_parameter('gnss_device', '/dev/ttyAMA0').value
        self.baud = self.declare_parameter('gnss_baud', 115200).value
        config = self.declare_parameter('config_file', configuration.default_path()).value
        path = self.declare_parameter('field_log_path', '').value
        name = 'field-simulation.sqlite3' if simulation else 'field.sqlite3'
        self.log = FieldLog(path or Path(config).resolve().with_name(name))
        self.policy = dict(DEFAULTS)
        self.source = None
        self.port_state = 'Simulation' if simulation else 'Waiting for receiver'
        self.stationary = False
        self.motion_at = 0
        self.stop = threading.Event()
        self.reader = None
        self.latest = {}
        self.latest_at = {}
        try:self.settings_snapshot = configuration.load(config)
        except (OSError,ValueError) as exc:self.settings_snapshot = dict(configuration_error=str(exc))
        self.session = MissionSession(Path(config).resolve().parent/'mission-sessions',
            self.settings_snapshot,software_version(config),simulation)
        self.last_inspection_phase = None
        self.logged_points = 0
        self.last_probe_phase = None
        self.loadcell_cursor = None
        self.loadcell_session = None
        self.log_error_reported = None
        self.state_pub = self.create_publisher(String, 'field/state', 1)
        retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.log_pub = self.create_publisher(String, 'field/entries', retained)
        self.create_subscription(String, 'field/mark', lambda msg: self.record(msg, True), 10)
        self.create_subscription(String, 'field/log', lambda msg: self.record(msg, False), 20)
        self.create_subscription(Bool, 'field/clear', self.clear, 10)
        self.create_subscription(String, 'operator/settings', self.settings, 1)
        self.create_subscription(String, 'operator/state', self.motion, 1)
        for topic,source in (('probe/web_state','probe'),('arm/web_state','arm'),
                ('detector/telemetry','detector'),('adc/state','adc'),
                ('power/state','power'),('cameras/state','cameras')):
            self.create_subscription(String,topic,lambda msg,name=source:self.telemetry(msg,name),1)
        for topic in ('operator/result','arm/result','probe/result','operator/settings/result','operator/command',
                      'detector/control_state','detector/state','adc/command_result'):
            self.create_subscription(String,topic,lambda msg,name=topic:self.source_event(msg,name),20)
        for message,topic in ((AdcSamples,'adc/samples'),(AdcSamples,'adc/filtered_samples'),(ServoState,'servo/state'),
                (ServoCommand,'servo/command'),(ActuatorCommand,'arm/command'),(ActuatorCommand,'probe/command'),
                (Odometry,'odometry/local'),(Twist,'cmd_vel'),(TwistWithCovarianceStamped,'mavros/local_position/velocity_local'),
                (RCIn,'mavros/rc/in'),(State,'mavros/state'),(Joy,'joy'),(Float32,'actuation/pump_percent')):
            self.create_subscription(message,topic,lambda msg,name=topic:self.raw_telemetry(msg,name),qos_profile_sensor_data)
        self.create_timer(.1,self.analysis_tick)
        self.create_timer(.5, self.tick)
        self.publish_entries()
        if self.declare_parameter('rtk_fix', False).value and not simulation:
            # The localization stack already owns the receiver in RTK mode.
            self.device = '/fix'
            self.create_subscription(NavSatFix, self.device, self.ros_fix, qos_profile_sensor_data)
        elif not simulation and self.device:
            self.reader = threading.Thread(target=self.read_receiver, daemon=True)
            self.reader.start()

    def ros_fix(self, msg):
        self.raw_telemetry(msg,'gnss/fix')
        stamp_ns = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds - stamp_ns) / 1e9
        valid = 0 <= age <= FRESH_S and msg.status.status >= 0 and abs(msg.latitude) <= 90 and abs(msg.longitude) <= 180
        ee, nn = msg.position_covariance[0], msg.position_covariance[4]
        known = msg.position_covariance_type != 0 and all(math.isfinite(n) and n >= 0 for n in (ee, nn))
        self.source = (time.monotonic() - max(0, age), dict(latitude=msg.latitude if valid else None,
            longitude=msg.longitude if valid else None, horizontal_accuracy_m=math.sqrt(ee + nn) if valid and known else None,
            fix='GNSS fix' if valid else 'No fix', satellites=None))

    def read_receiver(self):
        # Keep all serial I/O (including reconnects) outside the ROS executor.
        try:
            import serial
            from pyubx2 import UBXReader
        except ImportError:
            self.port_state = 'GNSS reader unavailable'
            return
        while not self.stop.is_set():
            try:
                with serial.Serial(self.device, self.baud, timeout=.4, exclusive=True) as port:
                    reader = UBXReader(port)
                    self.port_state = 'Waiting for GNSS data'
                    last_pvt = -float('inf')
                    while not self.stop.is_set():
                        raw, msg = reader.read()
                        now = time.monotonic()
                        if msg is None:
                            continue
                        self.session.write('gnss',dict(identity=msg.identity,raw_hex=raw.hex(),
                            fields={key:value for key,value in vars(msg).items() if not key.startswith('_')}),self.current())
                        if msg.identity == 'NAV-PVT':
                            last_pvt = now
                        elif now - last_pvt < FRESH_S:
                            continue
                        fix = position(msg)
                        if fix is not None:
                            self.source = (now, fix)
            except (OSError, ValueError, AttributeError) as exc:
                self.source = None
                self.port_state = f'Receiver unavailable · {exc}'
                self.stop.wait(2)

    def settings(self, msg):
        try:
            value=json.loads(msg.data)
            if value != self.settings_snapshot:
                self.settings_snapshot=value
                self.session.write('settings',value,self.current(),self.get_clock().now().nanoseconds)
            self.policy = validate(value['gnss'])
        except (KeyError, TypeError, ValueError):
            pass

    def motion(self, msg):
        state = json.loads(msg.data)
        self.latest['operator'],self.latest_at['operator']=clean(state,False),time.monotonic()
        inspection=state.get('inspection',{})
        phase=inspection.get('phase')
        if phase != self.last_inspection_phase:
            if phase=='PROBING':
                self.logged_points=0
                if inspection.get('probe_scan_mode')=='manual':
                    self.session.start_run('manual_probe_scan',self.settings_snapshot,self.current())
            self.analysis_event('inspection_phase',inspection)
            self.last_inspection_phase=phase
        points=inspection.get('points',[])
        for index in range(self.logged_points,len(points)):
            self.analysis_event('probe_point',dict(index=index,point=points[index],
                scan_mode=inspection.get('probe_scan_mode'),planned_angles=inspection.get('probe_angles_deg')))
        self.logged_points=len(points)
        robot, drive = state.get('robot', {}), state.get('drive', {})
        self.motion_at = time.monotonic()
        # With no wheel odometry, the control node still reports its own drive
        # commands. Direct transmitter control cannot supply that guarantee.
        drive_idle = not drive.get('cmd_linear_x') and not drive.get('cmd_angular_z')
        ros_mode = state.get('safety', {}).get('mode') in ('ros', 'simulation')
        self.stationary = drive_idle and ros_mode and (not robot.get('odometry_fresh') or robot.get('stationary') is True)
        if not self.stationary and getattr(self, 'waiting', False):
            if self.log.refine(None, time.time(), False):
                self.publish_entries()

    def current(self):
        sample = self.source
        if sample and time.monotonic() - sample[0] <= FRESH_S:
            return sample[1]
        return None

    def analysis_snapshot(self):
        now=time.monotonic()
        return dict(**self.latest,topic_age_s={key:round(now-at,6) for key,at in self.latest_at.items()},
            gnss=dict(fix=self.current(),last_fix=self.source[1] if self.source else None,
                      age_s=now-self.source[0] if self.source else None,receiver=self.port_state))

    def analysis_event(self,event,data):
        self.session.write('event',dict(event=event,data=data,snapshot=self.analysis_snapshot()),
            self.current(),self.get_clock().now().nanoseconds)

    def telemetry(self,msg,source):
        try:value=json.loads(msg.data)
        except (TypeError,ValueError):return
        self.latest[source],self.latest_at[source]=clean(value,False),time.monotonic()
        if source=='probe':
            probe=value.get('probe',{})
            marker=(probe.get('cycle_active'),probe.get('cycle_phase'),probe.get('contact_detected'),
                    (probe.get('last_result') or {}).get('timestamp'),probe.get('fault'))
            if marker != self.last_probe_phase:
                self.analysis_event('probe_phase',probe)
                self.last_probe_phase=marker
            cell=probe.get('loadcell',{})
            if cell.get('session') != self.loadcell_session:
                self.loadcell_cursor=None;self.loadcell_session=cell.get('session')
            samples=[]
            for sample in cell.get('samples',[]):
                stamp=sample.get('sample_ms')
                if type(stamp) is not int:continue
                if self.loadcell_cursor is not None and not 0 < ((stamp-self.loadcell_cursor)&0xffffffff) < 0x80000000:continue
                samples.append(sample);self.loadcell_cursor=stamp
            if samples:
                self.session.write('loadcell',dict(session=self.loadcell_session,
                    history_time_unix_s=probe.get('history_time_s'),samples=samples),self.current())

    def source_event(self,msg,source):
        try:data=json.loads(msg.data)
        except (TypeError,ValueError):return
        self.session.write('source',dict(source=source,data=data),self.current(),self.get_clock().now().nanoseconds)

    def raw_telemetry(self,msg,source):
        self.session.write('source',dict(source=source,data=message_to_ordereddict(msg)),
            self.current(),self.get_clock().now().nanoseconds)

    def analysis_tick(self):
        self.session.write('telemetry',self.analysis_snapshot(),self.current(),self.get_clock().now().nanoseconds)
        if self.session.error and self.session.error != self.log_error_reported:
            self.get_logger().error('Mission recording failed: '+self.session.error)
            self.log_error_reported=self.session.error

    def publish_entries(self):
        entries = self.log.entries()
        self.waiting = any(item.get('gps_status') == 'pending' for item in entries)
        self.log_pub.publish(String(data=json.dumps(dict(revision=uuid.uuid4().hex, entries=entries))))

    def record(self, msg, mark):
        try:
            payload = json.loads(msg.data)
            if payload.get('status')=='SEARCH SWEEP':
                self.session.start_run('search_sweep',self.settings_snapshot,self.current())
            result=self.latest.get('probe',{}).get('probe',{}).get('last_result') or {}
            if result.get('timestamp')==payload.get('timestamp'):
                for key in ('contact_counts','contact_source','contact_threshold_counts'):
                    payload[key]=result.get(key)
            payload['telemetry']=self.analysis_snapshot()
            self.analysis_event('mark' if mark else 'mission_event',payload)
            if self.log.record(payload, mark, self.current(), self.policy, time.time()):
                self.publish_entries()
        except (TypeError, ValueError) as exc:
            self.get_logger().error(f'Invalid mission event: {exc}')

    def clear(self, msg):
        if msg.data:
            self.analysis_event('summary_cleared',{})
            self.log.clear()
            self.publish_entries()

    def tick(self):
        fix = self.current()
        now = time.time()
        stationary = self.stationary and time.monotonic() - self.motion_at < 1
        if self.waiting and self.log.refine(fix, now, stationary):
            self.publish_entries()
        self.log.trace(fix, now)
        state = dict(receiver=fix['fix'] if fix else ('Stale fix' if self.source else self.port_state),
                     device=self.device, fix=fix, trail=self.log.trail(),recording=self.session.status())
        self.state_pub.publish(String(data=json.dumps(state, allow_nan=False)))

    def destroy_node(self):
        self.stop.set()
        if self.reader:
            self.reader.join(timeout=1)
        self.session.close()
        self.log.db.close()
        super().destroy_node()


if __name__ == '__main__':
    run_node(FieldRecorder)
