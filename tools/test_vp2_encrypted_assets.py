"""tools/vp2_encrypted_assets.py: the vectorised keystream equals the game's loop (mod027 368c90, mode 2, shift 5)."""
import os
import struct
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import vp2_encrypted_assets as enc  # noqa: E402


def reference(data, state):
    out = bytearray(data)
    for k in range(len(data) // 2):
        state = (enc.MULTIPLIER * state + enc.INCREMENT) & 0xffffffff
        h = struct.unpack_from('<H', data, 2 * k)[0] ^ ((state >> enc.SHIFT) & 0xffff)
        struct.pack_into('<H', out, 2 * k, h)
    return bytes(out)


class Keystream(unittest.TestCase):
    def test_matches_the_loop_across_blocks(self):
        data = bytes((i * 37 + 11) & 0xff for i in range(2 * enc.BLOCK + 1001))   # odd length, three blocks
        for state in (0x1677, 0x920ad5, 0):
            self.assertEqual(enc.decrypt(data, state), reference(data, state))

    def test_round_trip(self):
        data = b'SLZ\x03' + bytes(range(200))
        self.assertEqual(enc.decrypt(enc.decrypt(data, 0x3eca3), 0x3eca3), data)


if __name__ == '__main__':
    unittest.main()
