"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

class primitives_t(object):
    __slots__ = ["i8", "i16", "num_ranges", "i64", "ranges", "position", "orientation", "name", "enabled"]

    __typenames__ = ["int8_t", "int16_t", "int32_t", "int64_t", "int16_t", "float", "double", "string", "boolean"]

    __dimensions__ = [None, None, None, None, ["num_ranges"], [3], [4], None, None]

    def __init__(self):
        self.i8 = 0
        self.i16 = 0
        self.num_ranges = 0
        self.i64 = 0
        self.ranges = []
        self.position = [ 0.0 for dim0 in range(3) ]
        self.orientation = [ 0.0 for dim0 in range(4) ]
        self.name = ""
        self.enabled = False

    def encode(self):
        buf = BytesIO()
        buf.write(primitives_t._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">bhiq", self.i8, self.i16, self.num_ranges, self.i64))
        buf.write(struct.pack('>%dh' % self.num_ranges, *self.ranges[:self.num_ranges]))
        buf.write(struct.pack('>3f', *self.position[:3]))
        buf.write(struct.pack('>4d', *self.orientation[:4]))
        __name_encoded = self.name.encode('utf-8')
        buf.write(struct.pack('>I', len(__name_encoded)+1))
        buf.write(__name_encoded)
        buf.write(b"\0")
        buf.write(struct.pack(">b", self.enabled))

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != primitives_t._get_packed_fingerprint():
            raise ValueError("Decode error")
        return primitives_t._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = primitives_t()
        self.i8, self.i16, self.num_ranges, self.i64 = struct.unpack(">bhiq", buf.read(15))
        self.ranges = struct.unpack('>%dh' % self.num_ranges, buf.read(self.num_ranges * 2))
        self.position = struct.unpack('>3f', buf.read(12))
        self.orientation = struct.unpack('>4d', buf.read(32))
        __name_len = struct.unpack('>I', buf.read(4))[0]
        self.name = buf.read(__name_len)[:-1].decode('utf-8', 'replace')
        self.enabled = bool(struct.unpack('b', buf.read(1))[0])
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if primitives_t in parents: return 0
        tmphash = (0xc2731598914f36aa) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff) + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if primitives_t._packed_fingerprint is None:
            primitives_t._packed_fingerprint = struct.pack(">Q", primitives_t._get_hash_recursive([]))
        return primitives_t._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

