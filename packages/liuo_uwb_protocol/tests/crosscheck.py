"""Independent handbook Python encoder -> production C++ decoder/encoder."""
import base64
import random
import struct
import subprocess
import sys
from dataclasses import replace
from protocol_reference import Packet, Edge, encode, decode, crc16_ccitt_false
rng = random.Random(20261025)
p = Packet(7,1,0x10203040,7,1234,123456,123400,12,0,1,
           tuple(Edge(j,3,10000+j*123,160+j,150+j) for j in range(2,7)))
golden = b'~U:TFIBBwEFQDAgEAcAAADSBAAAQOIBAAjiAQAMAAABAgMGKAAAopgDA4EoAACjmQQD/CgAAKSaBQN3KQAApZsGA/IpAACmnEZn\n'
assert encode(p) == golden
valid = [golden]
assert [len(encode(replace(p,edges=p.edges[:n]))) for n in range(1,6)] == [60,68,80,92,100]
for _ in range(2000):
    flags = rng.randrange(8)
    reporter = rng.randint(1,254)
    ids = sorted(rng.sample([i for i in range(1,255) if i!=reporter], rng.randint(1,5)))
    edges = tuple(Edge(i,3,rng.randint(1,0x7fffff),rng.randrange(256),rng.randrange(256)) for i in ids)
    shards = rng.randint(1,51)
    q = Packet(flags,reporter,rng.randint(1,2**32-1),rng.randrange(2**32),rng.randrange(2**32),
               rng.randrange(2**32) if flags&2 else 0,rng.randrange(2**32) if flags&1 else 0,
               rng.randrange(65535) if flags&4 else 65535,rng.randrange(shards),shards,edges)
    wire = encode(q)
    assert decode(wire)==q
    valid.append(wire)
# Semantic mutations have a repaired CRC, so each rejection exercises fields.
invalid = []
def mutate(offset, data, repair=True):
    raw = bytearray(base64.b64decode(golden[3:-1]))
    raw[offset:offset+len(data)] = data
    if repair: raw[-2:] = struct.pack('<H',crc16_ccitt_false(raw[:-2]))
    invalid.append(b'~U:'+base64.b64encode(raw)+b'\n')
for offset, vals in [(0,[0]),(2,[0,2]),(3,[8,255]),(4,[0,255]),(5,[0,6]),
                     (28,[1,255]),(29,[0,52]),(30,[0,1,255]),(31,[0,2,4,255]),(38,[2])]:
    for val in vals: mutate(offset,bytes([val]))
mutate(6,b'\0'*4)
for d in [0,-1,0x800000,0x7fffffff]: mutate(32,struct.pack('<i',d))
mutate(26,b'\xff\xff')
for flags in [0,1,2,3,4,5,6]: mutate(3,bytes([flags]))
mutate(31,b'\x01')  # unavailable RSSI but nonzero codes
mutate(35,b'\x01',False)
invalid += [golden[:-1]+b'\r\n', golden[:-1]+b'X\n', b'~U:!!!!\n', b'~U:\n', b'not-envelope\n']
for n in range(3,len(golden)-1): invalid.append(golden[:n]+b'\n')
# Noncanonical padding bits on a one-edge packet.
one=encode(replace(p,edges=p.edges[:1])); invalid.append(one[:-4]+b'B==\n')
proc = subprocess.run([sys.argv[1]],input=b''.join(valid+invalid),stdout=subprocess.PIPE,check=True)
lines=proc.stdout.splitlines(keepends=True)
assert lines[:len(valid)] == valid
assert len(lines)==len(valid)+len(invalid)
for inp,out in zip(invalid,lines[len(valid):]): assert out.startswith(b'ERROR'), (inp,out)
print(f'PASS: golden bytes, 2000 seeded cross-language packets, {len(invalid)} malformed/semantic cases')
