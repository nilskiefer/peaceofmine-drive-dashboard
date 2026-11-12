"""Durable automatic recordings, timestamp fidelity and lossless exports."""
import csv
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from peaceofmine_operator.mission_session import MissionSession, export_chunks


class MissionSessionTest(unittest.TestCase):
    def test_automatic_header_run_settings_and_original_event_time_survive_flush(self):
        with tempfile.TemporaryDirectory() as directory:
            session=MissionSession(directory,{'threshold_counts':1500000},{'commit':'abc','describe':'v1'})
            first=session.run_id
            stamp=1760000000123456789
            with patch('peaceofmine_operator.mission_session.time.time_ns',return_value=stamp), \
                 patch('peaceofmine_operator.mission_session.time.monotonic_ns',return_value=900123):
                session.write('event',{'event':'contact','counts':750000,'invalid':float('nan')},
                              {'latitude':59.,'longitude':18.},ros_time_ns=456)
            session.start_run('manual_probe_scan',{'threshold_counts':1000000})
            session.close()
            self.assertIsNone(session.error)
            records=[json.loads(line) for line in session.path.read_text().splitlines()]
            self.assertEqual(records[0]['kind'],'session_start')
            self.assertEqual(records[0]['payload']['software']['describe'],'v1')
            contact=records[1]
            self.assertEqual(contact['timestamp_unix_ns'],stamp)
            self.assertEqual(contact['timestamp_utc'],'2025-10-09T08:53:20.123456789Z')
            self.assertEqual(contact['monotonic_ns'],900123)
            self.assertEqual(contact['ros_time_ns'],456)
            self.assertIsNone(contact['payload']['invalid'])
            self.assertEqual(contact['run_id'],first)
            run=records[2]
            self.assertNotEqual(run['run_id'],first)
            self.assertEqual(run['payload']['settings']['threshold_counts'],1000000)
            self.assertEqual(records[-1]['kind'],'session_end')

    def test_exports_keep_full_payload_and_skip_torn_last_record(self):
        with tempfile.TemporaryDirectory() as directory:
            session=MissionSession(directory,{}, {'commit':'abc'})
            session.write('telemetry',{'probe':{'probe':{'depth_mm':42.,'loadcell':{'net_counts':750000}}},
                'detector':{'detector':{'filtered_volts':2.4}},'nested':{'original':[1,2,3]}},
                {'latitude':59.,'longitude':18.,'horizontal_accuracy_m':.4})
            session.close()
            original=session.path.read_bytes()
            with session.path.open('ab') as handle:handle.write(b'{"incomplete":')
            self.assertEqual(b''.join(export_chunks(session.path,'jsonl')),original)
            rows=list(csv.DictReader(io.StringIO(b''.join(export_chunks(session.path,'csv')).decode())))
            row=next(row for row in rows if row['kind']=='telemetry')
            self.assertEqual(float(row['probe_depth_mm']),42.)
            self.assertEqual(float(row['probe_contact_counts']),750000.)
            self.assertEqual(json.loads(row['payload_json'])['nested']['original'],[1,2,3])


class MissionDownloadTest(unittest.IsolatedAsyncioTestCase):
    async def test_http_download_and_filename_boundary(self):
        from aiohttp.test_utils import TestClient, TestServer
        from unittest.mock import Mock
        import importlib.util
        script=Path(__file__).parents[1]/'scripts/operator_web_node.py'
        spec=importlib.util.spec_from_file_location('mission_web',script)
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        with tempfile.TemporaryDirectory() as directory:
            session=MissionSession(Path(directory)/'mission-sessions',{}, {})
            session.write('event',{'event':'contact','counts':55})
            session.close()
            node=Mock()
            node.get_parameter.side_effect=lambda name:Mock(value=str(Path(directory)/'config.json') if name=='config_file' else 'http://127.0.0.1:8081')
            node.connected_sockets.return_value=[]
            async with TestClient(TestServer(module.build_app(node))) as client:
                response=await client.get('/api/mission-session?format=jsonl')
                self.assertEqual(response.status,200)
                self.assertEqual(await response.read(),session.path.read_bytes())
                self.assertIn('attachment',response.headers['Content-Disposition'])
                response=await client.get('/api/mission-session?format=csv')
                self.assertEqual(response.status,200)
                self.assertIn('payload_json',await response.text())
                response=await client.get('/api/mission-session?filename=../config.json')
                self.assertEqual(response.status,400)
