"""LIUO Range Relay v1: host-side codec reference, not a UWB/ROS driver."""
from __future__ import annotations
import base64
import binascii
import math
import struct
from dataclasses import dataclass
from typing import Tuple

# < : little endian, no native alignment. Header=30 B, record=8 B.
HEADER = struct.Struct('<2sBBBBIIIIIHBB')
EDGE = struct.Struct('<BBiBB')
MAX_EDGES = 5
MAX_WIRE = 100
PREFIX = b'~U:'
SUFFIX = b'\n'
SYS_TIME_VALID = 1
LOCAL_TIME_VALID = 2
HOLD_AGE_VALID = 4
RANGE_VALID = 1
RSSI_VALID = 2

@dataclass(frozen=True)
class Edge:
    peer_id: int
    flags: int
    distance_mm: int
    fp_code: int
    rx_code: int

@dataclass(frozen=True)
class Packet:
    flags: int
    reporter_id: int
    boot_id: int
    tx_cycle_seq: int
    source_frame_seq: int
    source_local_ms: int
    source_system_ms: int
    hold_age_ms: int
    shard_index: int
    shard_count: int
    edges: Tuple[Edge, ...]

def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc

def _uint(name: str, value: int, bits: int) -> None:
    if not isinstance(value, int) or isinstance(value, bool) or not 0 <= value < (1 << bits):
        raise ValueError(f'{name}: expected uint{bits}')

def validate(p: Packet) -> None:
    if p.flags & ~7:
        raise ValueError('reserved packet flag bits are nonzero')
    if not 1 <= p.reporter_id <= 254:
        raise ValueError('reporter must be a fixed logical node in 1..254')
    for name in ('boot_id', 'tx_cycle_seq', 'source_frame_seq', 'source_local_ms', 'source_system_ms'):
        _uint(name, getattr(p, name), 32)
    if p.boot_id == 0:
        raise ValueError('boot_id=0 is reserved')
    if not 1 <= p.shard_count <= 51 or not 0 <= p.shard_index < p.shard_count:
        raise ValueError('invalid shard index/count')
    if not 1 <= len(p.edges) <= MAX_EDGES:
        raise ValueError('v1 range packet needs 1..5 edges; no synthetic heartbeat edge')
    _uint('hold_age_ms', p.hold_age_ms, 16)
    if p.flags & HOLD_AGE_VALID:
        if p.hold_age_ms == 0xFFFF:
            raise ValueError('known hold age cannot use unknown sentinel')
    elif p.hold_age_ms != 0xFFFF:
        raise ValueError('unknown hold age must be 0xFFFF')
    if not (p.flags & SYS_TIME_VALID) and p.source_system_ms != 0:
        raise ValueError('invalid system time must be encoded as zero')
    if not (p.flags & LOCAL_TIME_VALID) and p.source_local_ms != 0:
        raise ValueError('invalid local time must be encoded as zero')
    ids = []
    for e in p.edges:
        if not 1 <= e.peer_id <= 254 or e.peer_id == p.reporter_id:
            raise ValueError('invalid fixed peer id')
        if e.flags & ~3 or not (e.flags & RANGE_VALID):
            raise ValueError('invalid edge flags')
        if not 1 <= e.distance_mm <= 0x7FFFFF:
            raise ValueError('positive range must fit source int24 millimetres')
        _uint('fp_code', e.fp_code, 8)
        _uint('rx_code', e.rx_code, 8)
        if not (e.flags & RSSI_VALID) and (e.fp_code or e.rx_code):
            raise ValueError('unavailable RSSI must encode as zero with flag cleared')
        ids.append(e.peer_id)
    if ids != sorted(set(ids)):
        raise ValueError('peer IDs must be strictly ascending within a shard')

def encode(p: Packet) -> bytes:
    validate(p)
    raw = HEADER.pack(b'LR', 1, p.flags, p.reporter_id, len(p.edges),
                      p.boot_id, p.tx_cycle_seq, p.source_frame_seq,
                      p.source_local_ms, p.source_system_ms, p.hold_age_ms,
                      p.shard_index, p.shard_count)
    raw += b''.join(EDGE.pack(e.peer_id, e.flags, e.distance_mm, e.fp_code, e.rx_code) for e in p.edges)
    raw += struct.pack('<H', crc16_ccitt_false(raw))
    wire = PREFIX + base64.b64encode(raw) + SUFFIX
    if len(wire) > MAX_WIRE:
        raise ValueError('wire payload exceeds 100 bytes')
    return wire

def decode(wire: bytes) -> Packet:
    if not isinstance(wire, bytes):
        raise TypeError('wire must be bytes')
    if len(wire) > MAX_WIRE or not wire.startswith(PREFIX) or not wire.endswith(SUFFIX):
        raise ValueError('bad envelope or length')
    encoded = wire[len(PREFIX):-len(SUFFIX)]
    try:
        raw = base64.b64decode(encoded, validate=True)
    except (binascii.Error, ValueError) as exc:
        raise ValueError('invalid Base64') from exc
    if base64.b64encode(raw) != encoded:
        raise ValueError('non-canonical Base64')
    if len(raw) < HEADER.size + EDGE.size + 2:
        raise ValueError('truncated payload')
    fields = HEADER.unpack_from(raw)
    magic, version, flags, source, count, boot, cycle, seq, local, system, age, shard, shards = fields
    if magic != b'LR' or version != 1 or count > MAX_EDGES:
        raise ValueError('unknown magic/version or count')
    if len(raw) != HEADER.size + count * EDGE.size + 2:
        raise ValueError('length/count mismatch')
    if struct.unpack_from('<H', raw, len(raw)-2)[0] != crc16_ccitt_false(raw[:-2]):
        raise ValueError('CRC mismatch')
    edges = tuple(Edge(*EDGE.unpack_from(raw, HEADER.size + k*EDGE.size)) for k in range(count))
    p = Packet(flags, source, boot, cycle, seq, local, system, age, shard, shards, edges)
    validate(p)
    return p

def rssi_code(db: float) -> int:
    if not math.isfinite(db) or not -127.5 <= db <= 0:
        raise ValueError('RSSI outside representable vendor range')
    return int(round(-2.0 * db))

if __name__ == '__main__':
    import random
    assert HEADER.size == 30 and EDGE.size == 8
    assert crc16_ccitt_false(b'123456789') == 0x29B1
    p = Packet(7, 1, 0x10203040, 7, 1234, 123456, 123400, 12, 0, 1,
               tuple(Edge(j, 3, 10000+j*123, 160+j, 150+j) for j in range(2,7)))
    wire = encode(p)
    assert len(wire) == 100 and decode(wire) == p
    print('GOLDEN_ASCII_REPR=', repr(wire))
    print('GOLDEN_BINARY_HEX=', base64.b64decode(wire[3:-1]).hex(' '))
    print('WIRE_LENGTHS=', [len(encode(Packet(7,1,1,1,1,1,1,0,0,1,p.edges[:n]))) for n in range(1,6)])
    rng = random.Random(20261025)
    for _ in range(2000):
        n = rng.randint(1,5)
        edges = tuple(Edge(j,3,rng.randint(1,500000),rng.randrange(256),rng.randrange(256))
                      for j in range(2,2+n))
        q = Packet(7,1,rng.randint(1,2**32-1),rng.randrange(2**32),rng.randrange(2**32),
                   rng.randrange(2**32),rng.randrange(2**32),rng.randrange(65535),0,1,edges)
        assert decode(encode(q)) == q
    raw = bytearray(base64.b64decode(wire[3:-1]))
    raw[35] ^= 1
    corrupted = PREFIX + base64.b64encode(raw) + SUFFIX
    try:
        decode(corrupted)
    except ValueError:
        pass
    else:
        raise AssertionError('CRC corruption not rejected')
    for bad in (wire[:-1], wire+b'X', b'~U:!!!!\n', wire[:20]+b'\n'):
        try:
            decode(bad)
        except ValueError:
            pass
        else:
            raise AssertionError('malformed envelope not rejected')
    print('PASS: golden vector, 2000 randomized round trips, CRC and malformed input checks')
