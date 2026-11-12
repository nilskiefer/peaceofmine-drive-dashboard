import unittest
from pathlib import Path
import tempfile
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
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.link = Path(directory.name) / 'usb-Espressif-loadcell-if00'
        self.link.symlink_to('/dev/ttyACM0')
        for patcher in (patch('peaceofmine_operator.loadcell_serial.serial.Serial',return_value=self.connection),
                        patch('peaceofmine_operator.loadcell_serial.Path.glob',side_effect=lambda pattern:[self.link]),
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
        self.assertIs(self.reader.connection,self.connection)
        self.assertFalse(self.reader.snapshot()['fresh'])
        self.feed('300,1200,,\n')
        self.assertTrue(self.reader.snapshot()['fresh'])
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
        self.link.unlink()
        self.link.symlink_to('/dev/ttyACM1')
        self.now += 1.1;self.reader.poll()
        self.assertEqual(self.reader.device,str(self.link))
        self.assertEqual(self.connection.port,str(self.link))

    def test_boot_messages_and_partial_records_do_not_reconnect(self):
        self.feed('rst:0x15 (USB_UART_CHIP_RESET),boot:0xc (SPI_FAST_FLASH_BOOT)\n')
        self.assertFalse(self.reader.snapshot()['fresh'])
        self.feed('1255462,,\n')
        self.assertFalse(self.reader.snapshot()['fresh'])
        self.feed('800,-1255462,,\n900,-1255509,,\n')
        self.assertTrue(self.reader.snapshot()['fresh'])
        self.assertEqual(self.reader.snapshot()['raw_counts'],-1255509)
        self.assertIs(self.reader.connection,self.connection)
        self.assertFalse(self.connection.closed)
        self.assertEqual(self.reader.session,1)

    def test_missing_persistent_link_does_not_fall_back_to_numbered_port(self):
        self.reader.close()
        self.link.unlink()
        self.link.symlink_to('/dev/ttyACM99')
        self.now += 1.1;self.reader.poll()
        self.assertIsNone(self.reader.connection)
        self.assertIn('persistent USB link',self.reader.error)

    def test_ambiguous_boards_are_not_selected_by_enumeration_order(self):
        self.reader.close()
        other = self.link.parent / 'usb-Espressif-other-if00'
        other.symlink_to('/dev/ttyACM2')
        self.ports.append(SimpleNamespace(vid=0x303a,pid=0x1001,device='/dev/ttyACM2'))
        with patch('peaceofmine_operator.loadcell_serial.Path.glob',return_value=[self.link,other]):
            self.now += 1.1;self.reader.poll()
        self.assertIsNone(self.reader.connection)
        self.assertIn('only one ESP32',self.reader.error)

    def test_filtered_counts_are_separate_from_raw_and_invalid_filter_fails_closed(self):
        self.feed('100,1000,,,750.5\n')
        self.assertEqual(self.reader.snapshot()['raw_counts'],1000)
        self.assertEqual(self.reader.snapshot()['filtered_counts'],750.5)
        self.assertEqual(self.reader.snapshot(True)['samples'][0]['filtered_counts'],750.5)
        for value in ('nan','inf','-inf','8388607','bad',''):
            self.feed(f'200,1000,,,{value}\n')
            self.assertFalse(self.reader.snapshot()['fresh'])
        self.feed('300,2000,,,1500\n')
        self.assertTrue(self.reader.snapshot()['fresh'])
        self.assertEqual(self.reader.snapshot()['filtered_counts'],1500)

    def test_zero_averages_the_previous_ten_seconds_instead_of_latest_reading(self):
        self.assertFalse(self.reader.snapshot()['can_zero'])
        with self.assertRaisesRegex(ValueError,'Fresh'):self.reader.zero_average()
        self.feed('100,3000,,,2000\n')
        with self.assertRaisesRegex(ValueError,'10 seconds'):self.reader.zero_average()
        for index in range(2,103):
            # A late step changes the average, without replacing it by the
            # most recent sample. Raw and filtered channels stay distinct.
            filtered = 1000 if index < 100 else 2000
            self.feed(f'{index*100},3000,,,{filtered}\n')
        recent = self.reader.zero_window(self.now)
        self.assertTrue(self.reader.snapshot()['can_zero'])
        expected = sum(point[2] for point in recent)/len(recent)
        self.assertAlmostEqual(self.reader.zero_average(),expected)
        self.assertGreater(self.reader.zero_average(),1000)
        self.assertLess(self.reader.zero_average(),1100)
        self.assertNotEqual(self.reader.zero_average(),2000)

    def test_zero_rejects_a_stale_or_interrupted_window(self):
        for index in range(101):self.feed(f'{index*100},1000,,\n')
        self.assertTrue(self.reader.snapshot()['can_zero'])
        self.now += .51
        self.assertFalse(self.reader.snapshot()['can_zero'])
        with self.assertRaises(ValueError):self.reader.zero_average()
        self.reader.close()
        self.now += 1.1;self.reader.poll()
        self.feed('100,1000,,\n')
        self.assertFalse(self.reader.snapshot()['can_zero'])


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
