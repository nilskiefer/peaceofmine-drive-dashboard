"""Automatic, append-only analysis logs; filesystem I/O stays off ROS callbacks."""
import csv
from datetime import datetime, timezone
import io
import json
import math
import os
from pathlib import Path
import platform
import queue
import socket
import subprocess
import threading
import time
import uuid


HISTORY_KEYS = {'samples','history','pressure_samples','contact_samples','trail','sweep_samples','recent_points'}


def clean(value, histories=True):
    if isinstance(value,dict):
        return {str(k):clean(v,histories) for k,v in value.items() if histories or k not in HISTORY_KEYS}
    if isinstance(value,(list,tuple)):
        return [clean(v,histories) for v in value]
    if isinstance(value,float) and not math.isfinite(value):
        return None
    if value is None or isinstance(value,(str,int,float,bool)):
        return value
    if hasattr(value,'tolist'):
        return clean(value.tolist(),histories)
    return str(value)


def utc_stamp(stamp_ns):
    seconds,nanos=divmod(stamp_ns,1_000_000_000)
    return datetime.fromtimestamp(seconds,timezone.utc).strftime('%Y-%m-%dT%H:%M:%S')+f'.{nanos:09d}Z'


def software_version(config):
    def git(*args):
        try:
            return subprocess.run(['git','-C',str(Path(config).resolve().parent),*args],
                check=True,capture_output=True,text=True,timeout=2).stdout.strip()
        except (OSError,subprocess.SubprocessError):
            return None
    return dict(commit=os.environ.get('OPERATOR_GIT_COMMIT') or git('rev-parse','HEAD'),
                describe=os.environ.get('OPERATOR_GIT_DESCRIBE') or git('describe','--tags','--always','--dirty'),
                branch=os.environ.get('OPERATOR_GIT_BRANCH') or git('branch','--show-current'),
                dirty=os.environ.get('OPERATOR_GIT_DIRTY'),ros_distro=os.environ.get('ROS_DISTRO'),
                python=platform.python_version(),host=socket.gethostname())


class MissionSession:
    def __init__(self,directory,settings,software,simulation=False):
        self.session_id=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')+'-'+uuid.uuid4().hex[:8]
        self.path=Path(directory)/f'mission-{self.session_id}.jsonl'
        self.run_id=uuid.uuid4().hex
        self.software=software
        self.rows=self.dropped=0
        self.error=None
        self.queue=queue.Queue(maxsize=10000)
        self.lock=threading.RLock()
        self.ready=threading.Event()
        self.thread=threading.Thread(target=self._writer,daemon=True,name='mission-log-writer')
        self.thread.start()
        self.ready.wait(2)
        self.write('session_start',dict(settings=settings,software=software,simulation=simulation,
            clocks=dict(utc='System UTC at receipt',monotonic='Host monotonic nanoseconds; use for ordering',
                        source='Original ROS/GNSS/ESP32 timestamps retained in source messages'),
            units=dict(probe_contact='ADC counts; not a calibrated force',motor_load='percent',depth='mm')))

    def write(self,kind,payload,location=None,ros_time_ns=None):
        with self.lock:
            return self._write(kind,payload,location,ros_time_ns)

    def _write(self,kind,payload,location=None,ros_time_ns=None):
        if self.error:
            self.dropped+=1
            return False
        # Capture before queueing: disk scheduling must not shift event times.
        wall,mono=time.time_ns(),time.monotonic_ns()
        record=dict(schema_version=1,session_id=self.session_id,run_id=self.run_id,
                    timestamp_utc=utc_stamp(wall),timestamp_unix_ns=wall,monotonic_ns=mono,
                    ros_time_ns=ros_time_ns,kind=kind,location=clean(location),payload=clean(payload))
        try:
            self.queue.put_nowait(record)
            return True
        except queue.Full:
            self.dropped+=1
            return False

    def start_run(self,reason,settings,location=None):
        with self.lock:
            self.run_id=uuid.uuid4().hex
            self.write('run_start',dict(reason=reason,settings=settings,software=self.software),location)

    def status(self):
        return dict(session_id=self.session_id,filename=self.path.name,error=self.error,
                    rows=self.rows,dropped=self.dropped,queued=self.queue.qsize())

    def _writer(self):
        try:
            self.path.parent.mkdir(parents=True,exist_ok=True)
            with self.path.open('x',encoding='utf-8') as handle:
                self.ready.set()
                synced=time.monotonic()
                reported_drops=0
                while True:
                    try:
                        record=self.queue.get(timeout=1)
                    except queue.Empty:
                        handle.flush();os.fsync(handle.fileno());synced=time.monotonic()
                        continue
                    if record is None:
                        break
                    if self.dropped != reported_drops:
                        record['dropped_records_total']=self.dropped
                        reported_drops=self.dropped
                    handle.write(json.dumps(record,allow_nan=False,separators=(',',':'))+'\n')
                    self.rows+=1
                    if record['kind'] in ('session_start','run_start','event','settings','session_end') or time.monotonic()-synced>=1:
                        handle.flush();os.fsync(handle.fileno());synced=time.monotonic()
                handle.flush();os.fsync(handle.fileno())
        except (OSError,ValueError,TypeError) as exc:
            self.error=str(exc)
            self.ready.set()

    def close(self):
        self.write('session_end',dict(rows=self.rows,dropped=self.dropped))
        if self.thread.is_alive():
            try:self.queue.put(None,timeout=2)
            except queue.Full:self.error='Mission log queue did not drain at shutdown'
            self.thread.join(timeout=5)
            if self.thread.is_alive():self.error='Mission log flush did not finish at shutdown'


def export_chunks(path,format):
    """Bound the export at click time and omit any incomplete trailing record."""
    buffer=io.StringIO()
    writer=csv.writer(buffer)
    if format=='csv':
        writer.writerow(['timestamp_utc','monotonic_ns','session_id','run_id','kind',
            'latitude','longitude','horizontal_accuracy_m','probe_depth_mm','probe_contact_counts',
            'detector_volts','event','payload_json','record_json'])
    with Path(path).open('rb') as handle:
        end=os.fstat(handle.fileno()).st_size
        while handle.tell()<end:
            line=handle.readline()
            if handle.tell()>end or not line.endswith(b'\n'):break
            if format=='jsonl':
                buffer.write(line.decode('utf-8'))
            else:
                record=json.loads(line)
                payload=record.get('payload') or {}
                snapshot=payload.get('snapshot',payload if record['kind']=='telemetry' else {})
                probe=snapshot.get('probe',{}).get('probe',{})
                detector=snapshot.get('detector',{}).get('detector',{})
                packet=detector.get('sensor',{}).get('packet') or {}
                detector_volts=detector.get('filtered_volts')
                if detector_volts is None and packet.get('filtered_adc') is not None:
                    detector_volts=packet['filtered_adc']*detector.get('sensor',{}).get('reference_voltage',3.3)/255
                location=record.get('location') or {}
                writer.writerow([record.get('timestamp_utc'),record.get('monotonic_ns'),record.get('session_id'),
                    record.get('run_id'),record.get('kind'),location.get('latitude'),location.get('longitude'),
                    location.get('horizontal_accuracy_m'),probe.get('depth_mm'),probe.get('loadcell',{}).get('net_counts'),
                    detector_volts,payload.get('event'),json.dumps(payload,separators=(',',':')),
                    json.dumps(record,separators=(',',':'))])
            if buffer.tell()>=65536:
                yield buffer.getvalue().encode('utf-8');buffer.seek(0);buffer.truncate()
    if buffer.tell():yield buffer.getvalue().encode('utf-8')
