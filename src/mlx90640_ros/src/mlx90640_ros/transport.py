"""Shared ESP32 serial/replay streams; no discovery or implicit device access."""
import math
import struct
import time

from . import core
from .protocol import PacketParser

EEPROM = 0x01
FRAME = 0x02
STATUS = 0x03


def positive_rate(value):
    value = float(value)
    if not math.isfinite(value) or value <= 0:
        raise ValueError("replay_rate must be finite and positive")
    return value


class _Stream:
    def __init__(self, stop=None, log=None, clock=None, sleep=None):
        self.stop = stop or (lambda: False)
        self.log = log or (lambda level, message: None)
        self.clock = clock or time.monotonic
        self.sleep = sleep or time.sleep

    def _wait(self, seconds):
        deadline = self.clock() + seconds
        while not self.stop():
            remaining = deadline - self.clock()
            if remaining <= 0:
                return True
            self.sleep(min(remaining, 0.05))
        return False


class SerialTransport(_Stream):
    """Yield fresh complete Samples, reconnecting only the explicitly named port.

    ``serial_factory``, clocks, and the stop callback are injectable for tests.
    ee_ready is transport state, not an assumption about Decoder internals.
    """
    def __init__(self, port, decoder, baud=460800, serial_factory=None,
                 parser_factory=None, **kwargs):
        super().__init__(**kwargs)
        if not isinstance(port, str) or not port.strip():
            raise ValueError("an explicit, non-empty serial port is required")
        self.port = port.strip()
        self.baud = int(baud)
        if self.baud <= 0:
            raise ValueError("baud must be positive")
        self.decoder = decoder
        self.serial_factory = serial_factory
        self.parser_factory = parser_factory or PacketParser
        self.serial = None
        self.parser = self.parser_factory()
        self.ee_ready = False
        self._last_request = float("-inf")
        self._last_frame = self.clock()
        self._last_parser_warning = float("-inf")
        self._parser_errors = [0, 0]

    def request_eeprom(self):
        """Discard partial temperatures before requesting fresh calibration."""
        self.decoder.reset()
        self.ee_ready = False
        # Keep the last send time: repeated invalid replies must not bypass
        # the one-second EEPROM request rate limit.
        self._last_frame = self.clock()

    def close(self):
        serial, self.serial = self.serial, None
        if serial is not None:
            try:
                serial.close()
            except Exception as exc:
                self.log("warning", "serial close failed: %s" % exc)
        self.parser = self.parser_factory()
        self.request_eeprom()

    def _status(self, payload):
        if len(payload) == 11:
            ms, frames, errors, flags = struct.unpack("<IIHB", payload)
            self.log("info", "ESP32 STATUS ms=%d frames=%d errors=%d flags=%d" %
                     (ms, frames, errors, flags))
        else:
            self.log("warning", "ESP32 STATUS unexpected payload: %s" % payload.hex())

    def samples(self):
        # Importing this module and constructing a transport never opens a port.
        factory = self.serial_factory
        serial_errors = (OSError,)
        if factory is None:
            import serial
            factory = serial.Serial
            serial_errors = (OSError, serial.SerialException)
        try:
            while not self.stop():
                try:
                    if self.serial is None:
                        self.parser = self.parser_factory()
                        self._last_request = float("-inf")
                        self.request_eeprom()
                        self.serial = factory(port=self.port, baudrate=self.baud,
                                              timeout=0.2, write_timeout=0.2)
                        self.serial.reset_input_buffer()
                        self.log("info", "connected to %s at %d baud" % (self.port, self.baud))
                    now = self.clock()
                    if self.ee_ready and now - self._last_frame >= 3.0:
                        self.log("warning", "no valid frame for 3 seconds; requesting EEPROM")
                        self.parser = self.parser_factory()
                        self.request_eeprom()
                    if not self.ee_ready and now - self._last_request >= 1.0:
                        self.serial.write(b"\x01")
                        self._last_request = now
                    data = self.serial.read(4096)
                    before = (getattr(self.parser, "crc_errors", 0),
                              getattr(self.parser, "invalid_headers", 0))
                    packets = self.parser.feed(data)
                    after = (getattr(self.parser, "crc_errors", 0),
                             getattr(self.parser, "invalid_headers", 0))
                    for index in range(2):
                        self._parser_errors[index] += after[index] - before[index]
                    if any(self._parser_errors) and self.clock() - self._last_parser_warning >= 1.0:
                        self.log("warning", "UART rejected packets: CRC errors=%d invalid headers=%d" %
                                 tuple(self._parser_errors))
                        self._parser_errors = [0, 0]
                        self._last_parser_warning = self.clock()
                    for typ, seq, payload in packets:
                        if self.stop():
                            break
                        if typ == STATUS:
                            self._status(payload)
                        elif typ == EEPROM:
                            try:
                                self.decoder.load_eeprom(payload)
                                self.ee_ready = True
                                self._last_frame = self.clock()
                                self.log("info", "EEPROM calibration ready")
                            except ValueError as exc:
                                self.log("warning", "invalid EEPROM: %s" % exc)
                                self.request_eeprom()
                        elif typ == FRAME and self.ee_ready:
                            try:
                                sample = self.decoder.process_frame(payload)
                                if not getattr(self.decoder, "ready", True):
                                    self.log("warning", "decoder reset detected; requesting EEPROM")
                                    self.request_eeprom()
                                    continue
                                self._last_frame = self.clock()
                            except ValueError as exc:
                                # Includes board reset / timestamp or sequence discontinuity.
                                self.log("warning", "frame rejected; recalibrating: %s" % exc)
                                self.request_eeprom()
                                continue
                            if sample is not None:
                                yield sample
                except serial_errors as exc:
                    self.log("warning", "serial error: %s; retrying in 2 seconds" % exc)
                    self.close()
                    if not self._wait(2.0):
                        break
        finally:
            self.close()


class ReplayTransport(_Stream):
    """Replay captured subpages at replay_rate Hz (default 2 => 1 full image/s)."""
    def __init__(self, capture_file, decoder, replay_rate=2.0, loop=False, **kwargs):
        super().__init__(**kwargs)
        self.capture_file = capture_file
        self.decoder = decoder
        self.replay_rate = positive_rate(replay_rate)
        self.loop = bool(loop)
        self._reload = False

    def request_eeprom(self):
        self._reload = True

    def close(self):
        self.decoder.reset()

    def samples(self):
        eeprom, frames = core.load_capture(self.capture_file)
        if not frames:
            raise ValueError("capture contains no frames")
        # The capture has no timestamps: retain its nominal 2 Hz device time.
        # replay_rate changes wall-clock speed only, never decoder gap semantics.
        interval_ms = 500
        try:
            while not self.stop():
                self.decoder.reset()
                self.decoder.load_eeprom(eeprom)
                self._reload = False
                for index, words in enumerate(frames):
                    if self.stop():
                        return
                    if self._reload:
                        self.decoder.reset()
                        self.decoder.load_eeprom(eeprom)
                        self._reload = False
                    sample = self.decoder.process_frame(
                        core.capture_payload(words, index, interval_ms))
                    if sample is not None:
                        yield sample
                    if not self._wait(1.0 / self.replay_rate):
                        return
                if not self.loop:
                    return
        finally:
            self.close()
