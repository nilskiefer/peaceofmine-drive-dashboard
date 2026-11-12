import unittest
from types import SimpleNamespace
from unittest.mock import patch

from peaceofmine_operator.loadcell_serial import LoadCellSerial, parse_sample
from peaceofmine_operator.probe_contact_settings import DEFAULTS, validate


class Connection:
    def __init__(self, **kwargs):
        self.data = b''
        self.writes = []
        self.closed = False

    @property
    def in_waiting(self):
        return len(self.data)

    def open(self): pass
    def close(self): self.closed = True
    def reset_input_buffer(self): self.data = b''
    def write(self, data): self.writes.append(data); return len(data)
    def read(self, size):
        result, self.data = self.data[:size], self.data[size:]
        return result


class LoadCellSerialTest(unittest.TestCase):
    def setUp(self):
        self.now = 10.
        self.reader = LoadCellSerial(clock=lambda:self.now)
        self.connection = Connection()
        self.ports = [SimpleNamespace(vid=0x303a,pid=0x1001,device='/dev/ttyACM0')]
        for patcher in (patch('peaceofmine_operator.loadcell_serial.serial.Serial',return_value=self.connection),
                        patch('peaceofmine_operator.loadcell_serial.list_ports.comports',side_effect=lambda:self.ports)):
            patcher.start();self.addCleanup(patcher.stop)
        self.reader.poll()

    def feed(self,text):
        self.connection.data += text.encode('ascii')
        self.now += .1
        self.reader.poll()

    def test_reads_raw_counts_without_handshake_or_mass_calibration(self):
        self.feed('ESP-ROM:boot\n100,1000,,\n')
        self.assertTrue(self.reader.snapshot()['fresh'])
        self.assertEqual(self.reader.snapshot()['raw_counts'],1000)
        self.feed('200,1100,10,nan\n')
        self.assertEqual(self.reader.snapshot()['raw_counts'],1100)
        self.assertEqual(self.connection.writes,[])
        self.assertEqual([v['raw_counts'] for v in self.reader.snapshot(True)['samples']],[1000,1100])

    def test_duplicate_and_stale_readings_do_not_invent_data(self):
        self.feed('100,1000,,\n')
        received=self.reader.received
        self.feed('100,1000,,\n')
        self.assertEqual(self.reader.received,received)
        self.assertEqual(len(self.reader.snapshot(True)['samples']),1)
        self.now += .51
        self.assertFalse(self.reader.snapshot()['fresh'])
        self.assertIsNone(self.reader.snapshot()['raw_counts'])

    def test_sensor_error_and_invalid_csv_fail_closed(self):
        self.feed('100,1000,,\n')
        self.feed('# ERROR hx711_timeout\n')
        self.assertFalse(self.reader.snapshot()['fresh'])
        self.assertIsNone(self.reader.snapshot()['raw_counts'])
        self.feed('200,broken,,\n')
        self.assertIsNone(self.reader.connection)
        for line in ('1,8388607,,','1,-8388608,,','-1,1,,','broken'):
            with self.subTest(line=line),self.assertRaises(ValueError):parse_sample(line)

    def test_rollover_works_and_device_reset_clears_old_samples(self):
        self.feed('4294967290,1000,,\n')
        self.feed('50,1100,,\n')
        self.assertEqual(self.reader.snapshot()['raw_counts'],1100)
        self.feed('10,1200,,\n')
        self.assertIsNone(self.reader.connection)
        self.assertEqual(self.reader.snapshot(True)['samples'],[])

    def test_delayed_frames_and_large_backlog_are_rejected(self):
        self.feed('100,1000,,\n')
        for _ in range(10):
            self.now += .1;self.reader.poll()
        self.feed('200,1000,,\n')
        self.assertFalse(self.reader.snapshot()['fresh'])
        self.assertIn('Delayed',self.reader.error)
        self.now += 1.1;self.reader.poll()
        self.connection.data=b'x'*4097
        self.reader.poll()
        self.assertIn('backlog',self.reader.error)

    def test_reconnects_when_esp32_is_plugged_in(self):
        self.reader.close()
        self.ports.clear()
        self.now += 1.1;self.reader.poll()
        self.assertIsNone(self.reader.connection)
        self.ports.append(SimpleNamespace(vid=0x303a,pid=0x1001,device='/dev/ttyACM1'))
        self.now += 1.1;self.reader.poll()
        self.assertEqual(self.reader.device,'/dev/ttyACM1')


class ContactSettingsTest(unittest.TestCase):
    def test_only_source_threshold_and_zero_are_saved(self):
        self.assertEqual(set(DEFAULTS),{'source','threshold_counts','zero_counts'})
        self.assertIsNone(validate({**DEFAULTS,'source':'esp32'})['threshold_counts'])
        good=dict(source='esp32',threshold_counts=50.,zero_counts=1234.)
        self.assertEqual(validate(good),good)
        for key,value in [('threshold_counts',True),('threshold_counts',0),('threshold_counts',float('nan')),
                          ('zero_counts',True),('zero_counts',8388607)]:
            with self.subTest(key=key,value=value),self.assertRaises(ValueError):validate({**good,key:value})


if __name__ == '__main__':unittest.main()
