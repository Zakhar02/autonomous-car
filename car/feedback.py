"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

class feedback(object):
    __slots__ = ["x", "y", "theta"]

    __typenames__ = ["double", "double", "double"]

    __dimensions__ = [None, None, None]

    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def encode(self):
        buf = BytesIO()
        buf.write(feedback._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">ddd", self.x, self.y, self.theta))

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != feedback._get_packed_fingerprint():
            raise ValueError("Decode error")
        return feedback._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = feedback()
        self.x, self.y, self.theta = struct.unpack(">ddd", buf.read(24))
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if feedback in parents: return 0
        tmphash = (0x7491c1074c104593) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff) + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if feedback._packed_fingerprint is None:
            feedback._packed_fingerprint = struct.pack(">Q", feedback._get_hash_recursive([]))
        return feedback._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)
