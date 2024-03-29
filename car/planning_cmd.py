"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

class planning_cmd(object):
    __slots__ = ["timestamp", "xd", "yd", "thetad", "vd", "dthetad"]

    __typenames__ = ["double", "double", "double", "double", "double", "double"]

    __dimensions__ = [None, None, None, None, None, None]

    def __init__(self):
        self.timestamp = 0.0
        self.xd = 0.0
        self.yd = 0.0
        self.thetad = 0.0
        self.vd = 0.0
        self.dthetad = 0.0

    def encode(self):
        buf = BytesIO()
        buf.write(planning_cmd._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">dddddd", self.timestamp, self.xd, self.yd, self.thetad, self.vd, self.dthetad))

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != planning_cmd._get_packed_fingerprint():
            raise ValueError("Decode error")
        return planning_cmd._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = planning_cmd()
        self.timestamp, self.xd, self.yd, self.thetad, self.vd, self.dthetad = struct.unpack(">dddddd", buf.read(48))
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if planning_cmd in parents: return 0
        tmphash = (0xf8e82a41549baa42) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff) + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if planning_cmd._packed_fingerprint is None:
            planning_cmd._packed_fingerprint = struct.pack(">Q", planning_cmd._get_hash_recursive([]))
        return planning_cmd._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

