"""Private USA ELF signature tests; no game bytes are checked into this file."""
import sys
import struct
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from vp2_bozo_prepare import elf_parts
from vp2_bozo_kernel import bindings, GUARDS, INSTALLER, QUERY

IMAGE=Path(sys.argv.pop(1))


class KernelGuardTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.data=IMAGE.read_bytes()
        _,cls.segments,_=elf_parts(cls.data)

    def test_valid_binding_ranges(self):
        declarations=bindings(self.data).splitlines()
        expected=set(range(*INSTALLER,4))|set(range(*QUERY,4))
        self.assertEqual({int(line.split()[1],0) for line in declarations if line.startswith('stub ')},expected)
        self.assertNotIn('stub 0x141548',declarations)
        self.assertIn('native 0x141448',declarations)
        self.assertIn('native 0x141410',declarations)
        self.assertNotIn('native 0x14142c',declarations) # jal: engine prohibits inline control hooks

    def test_every_guarded_word_rejects_mutation(self):
        for lo,hi in GUARDS:
            segment=next(s for s in self.segments if s[0]==1 and s[2]<=lo<hi<=s[2]+s[4])
            for pc in range(lo,hi,4):
                data=bytearray(self.data)
                data[segment[1]+pc-segment[2]]^=1
                with self.subTest(pc=hex(pc)),self.assertRaises(ValueError):
                    bindings(data)

    def test_invalid_or_truncated_image_rejects(self):
        for data in [b'',bytes(80),self.data[:128]]:
            with self.assertRaises((ValueError,struct.error)):
                bindings(data)


if __name__=='__main__': unittest.main()
