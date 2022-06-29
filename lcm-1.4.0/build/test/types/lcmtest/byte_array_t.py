"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

class byte_array_t(object):
    __slots__ = ["num_bytes", "data"]

    __typenames__ = ["int32_t", "byte"]

    __dimensions__ = [None, ["num_bytes"]]

    def __init__(self):
        self.num_bytes = 0
        self.data = ""

    def encode(self):
        buf = BytesIO()
        buf.write(byte_array_t._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">i", self.num_bytes))
        buf.write(bytearray(self.data[:self.num_bytes]))

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != byte_array_t._get_packed_fingerprint():
            raise ValueError("Decode error")
        return byte_array_t._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = byte_array_t()
        self.num_bytes = struct.unpack(">i", buf.read(4))[0]
        self.data = buf.read(self.num_bytes)
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if byte_array_t in parents: return 0
        tmphash = (0x870c7477e270debf) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff) + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if byte_array_t._packed_fingerprint is None:
            byte_array_t._packed_fingerprint = struct.pack(">Q", byte_array_t._get_hash_recursive([]))
        return byte_array_t._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

