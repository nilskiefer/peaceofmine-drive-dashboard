"""Nonblocking HX711 USB transport; never substitute motor/ADC data on failure."""
from collections import deque
from pathlib import Path
import math
import time
import serial
from serial.tools import list_ports

MAX_AGE = .5
ZERO_WINDOW_S = 10.


def parse_sample(line):
    parts = line.split(',')
    if len(parts) not in (4, 5) or not parts[0].isdigit():
        raise ValueError('Invalid HX711 CSV record')
    stamp, raw = int(parts[0]), int(parts[1])
    if not 0 <= stamp <= 0xffffffff or not -8388608 < raw < 8388607:
        raise ValueError('HX711 sample outside ADC range')
    filtered = float(parts[4]) if len(parts) == 5 else None
    if filtered is not None and (not math.isfinite(filtered) or not -8388608 < filtered < 8388607):
        raise ValueError('Invalid HX711 filtered reading')
    return dict(ms=stamp, raw_counts=raw, filtered_counts=filtered)


class LoadCellSerial:
    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.connection = None
        self.device = ''
        self.sample = None
        self.received = self.last_poll = None
        self.zero_since = None
        self.last_ms = None
        self.elapsed_ms = 0
        self.sample_offset = None
        self.session = 0
        self.buffer = bytearray()
        self.next_connect = 0.
        self.error = 'Select ESP32 to connect the load cell'
        self.samples = deque(maxlen=3000)

    def close(self, reason='Load cell disconnected'):
        if self.connection is not None:
            self.connection.close()
        self.connection = None
        self.sample = self.received = self.last_ms = self.last_poll = None
        self.zero_since = None
        self.buffer.clear()
        self.samples.clear()
        self.elapsed_ms = 0
        self.sample_offset = None
        self.error = reason

    def connect(self):
        ports = {p.device for p in list_ports.comports() if (p.vid,p.pid) == (0x303a,0x1001)}
        if not ports:
            raise ValueError('ESP32 load cell not connected')
        paths = sorted(str(path) for path in Path('/dev/serial/by-id').glob('*')
                       if str(path.resolve()) in ports)
        if not paths:
            raise ValueError('ESP32 persistent USB link unavailable (/dev/serial/by-id)')
        if len(paths) != 1:
            raise ValueError('Connect only one ESP32 load cell')
        connection = serial.Serial(port=None, baudrate=115200, timeout=0, exclusive=True)
        connection.dtr = connection.rts = False
        connection.port = paths[0]
        try:
            connection.open()
            connection.reset_input_buffer()
        except (serial.SerialException, OSError):
            connection.close()
            raise
        self.connection = connection
        self.device = paths[0]
        self.session += 1
        self.error = None

    def line(self, text):
        if text.startswith('# ERROR'):
            self.received = None
            self.zero_since = None
            self.error = text[2:]
            return
        if text.startswith('#') or ',' not in text:
            return
        try:
            sample = parse_sample(text)
        except ValueError as exc:
            # ROM boot messages contain commas, and an initial USB read can
            # start midway through a record. Closing/reopening on either can
            # reset the ESP32 again and leave it in a reconnect loop.
            self.received = None
            self.zero_since = None
            self.error = str(exc)
            return
        if self.last_ms is not None:
            step = (sample['ms'] - self.last_ms) & 0xffffffff
            if step == 0:
                return
            if step >= 0x80000000:
                raise ValueError('ESP32 restarted; reconnecting')
            self.elapsed_ms += step
        offset = self.clock() - self.elapsed_ms / 1000
        self.sample_offset = offset if self.sample_offset is None else min(self.sample_offset, offset)
        if offset - self.sample_offset >= MAX_AGE:
            raise ValueError('Delayed HX711 data; reconnecting')
        now = self.clock()
        if self.received is None or now-self.received >= MAX_AGE:
            self.zero_since = now
        self.sample, self.last_ms = sample, sample['ms']
        self.received, self.error = now, None
        self.samples.append((self.received, sample['raw_counts'], sample['filtered_counts']))

    def poll(self):
        now = self.clock()
        try:
            if self.connection is not None and self.last_poll is not None and now-self.last_poll >= MAX_AGE:
                raise ValueError('USB reader stalled; discarding queued readings')
            self.last_poll = now
            if self.connection is None:
                if now < self.next_connect:
                    return
                self.next_connect = now + 1.
                self.connect()
            if self.connection.in_waiting > 4096:
                raise ValueError('HX711 serial backlog; discarding stale readings')
            for byte in self.connection.read(4096):
                if byte == 10:
                    self.line(self.buffer.decode('ascii').rstrip('\r'))
                    self.buffer.clear()
                else:
                    self.buffer.append(byte)
                    if len(self.buffer) > 512:
                        raise ValueError('HX711 serial line too long')
        except (serial.SerialException, OSError, ValueError, UnicodeError) as exc:
            self.close(str(exc))
            self.next_connect = now + 1.

    def zero_window(self, now):
        if self.zero_since is None or now-self.zero_since+1e-9 < ZERO_WINDOW_S:
            return []
        samples = [sample for sample in self.samples if now-ZERO_WINDOW_S <= sample[0] <= now]
        if (len(samples) < 2 or samples[0][0] > now-ZERO_WINDOW_S+MAX_AGE
                or now-samples[-1][0] >= MAX_AGE
                or any(right[0]-left[0] >= MAX_AGE for left,right in zip(samples,samples[1:]))):
            return []
        return samples

    def zero_average(self):
        now = self.clock()
        if self.connection is None or self.received is None or now-self.received >= MAX_AGE:
            raise ValueError('Fresh load-cell reading required to zero')
        samples = self.zero_window(now)
        if not samples:
            raise ValueError('Keep the probe unloaded for 10 seconds of continuous readings before zeroing')
        return sum(sample[1] if sample[2] is None else sample[2] for sample in samples)/len(samples)

    def snapshot(self, include_history=False):
        now = self.clock()
        fresh = bool(self.connection is not None and self.received is not None
                     and now-self.received < MAX_AGE)
        state = dict(connected=self.connection is not None, fresh=fresh,
                     raw_counts=self.sample['raw_counts'] if fresh else None,
                     filtered_counts=self.sample['filtered_counts'] if fresh else None,
                     error=self.error, device=self.device,
                     sample_ms=self.last_ms, session=self.session,
                     can_zero=bool(fresh and self.zero_window(now)))
        if include_history:
            state['samples'] = [dict(x=round(at-now,4), raw_counts=raw, filtered_counts=filtered)
                                for at,raw,filtered in self.samples if now-at <= 30]
        return state
