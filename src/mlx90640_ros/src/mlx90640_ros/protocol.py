"""ESP32 UART framing, independent of ROS and serial-port libraries.

AA 55 | type:u8 | seq:u8 | payload_length:u16le | payload | crc:u16le.
CRC16-CCITT-FALSE covers type through payload, not the sync bytes.
"""

import struct

TYPE_EEPROM = 0x01
TYPE_FRAME = 0x02
TYPE_STATUS = 0x03
PAYLOAD_LENGTHS = {TYPE_EEPROM: 1664, TYPE_FRAME: 1674, TYPE_STATUS: 11}
SYNC = b"\xaa\x55"
MAX_PACKET_SIZE = max(PAYLOAD_LENGTHS.values()) + 8


def crc16(data):
    """Return CRC16-CCITT-FALSE (poly 0x1021, init 0xffff, no reflection)."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ (0x1021 if crc & 0x8000 else 0)) & 0xFFFF
    return crc


def encode_packet(typ, seq, payload):
    """Encode a known packet type; reject lengths the ESP32 cannot emit."""
    if typ not in PAYLOAD_LENGTHS:
        raise ValueError("unknown packet type: {!r}".format(typ))
    if not isinstance(seq, int) or not 0 <= seq <= 255:
        raise ValueError("sequence must be an unsigned byte")
    if len(payload) != PAYLOAD_LENGTHS[typ]:
        raise ValueError("incorrect payload length for packet type {}".format(typ))
    body = struct.pack("<BBH", typ, seq, len(payload)) + bytes(payload)
    return SYNC + body + struct.pack("<H", crc16(body))


class PacketParser:
    """Incremental, bounded parser; corrupt candidates resync one byte at a time.

    Unknown types and incorrect *type-specific* lengths are discarded as soon
    as the six-byte header arrives. Retained data never exceeds one packet.
    """

    def __init__(self):
        self._buffer = bytearray()
        self.crc_errors = 0
        self.invalid_headers = 0

    def feed(self, data):
        packets = []
        # Bound temporary storage too, even when a caller supplies huge noise.
        for offset in range(0, len(data), MAX_PACKET_SIZE):
            self._buffer.extend(data[offset:offset + MAX_PACKET_SIZE])
            while self._buffer:
                start = self._buffer.find(SYNC)
                if start < 0:
                    self._buffer[:] = b"\xaa" if self._buffer[-1] == 0xAA else b""
                    break
                if start:
                    del self._buffer[:start]
                if len(self._buffer) < 6:
                    break
                typ, seq, length = struct.unpack_from("<BBH", self._buffer, 2)
                if PAYLOAD_LENGTHS.get(typ) != length:
                    self.invalid_headers += 1
                    del self._buffer[0]
                    continue
                size = length + 8
                if len(self._buffer) < size:
                    break
                expected = struct.unpack_from("<H", self._buffer, size - 2)[0]
                if crc16(self._buffer[2:size - 2]) != expected:
                    self.crc_errors += 1
                    del self._buffer[0]
                    continue
                packets.append((typ, seq, bytes(self._buffer[6:size - 2])))
                del self._buffer[:size]
        return packets
