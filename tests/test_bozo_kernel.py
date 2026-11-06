"""Private USA ELF signature tests; no game bytes are checked into this file."""
import sys
import struct
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from vp2_bozo_prepare import elf_parts
from vp2_bozo_kernel import bindings, game_adaptations, frame_adaptations, game_entries, native_game_sites, GUARDS, RETIRED_RANGES

IMAGE=Path(sys.argv.pop(1))


class KernelGuardTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.data=IMAGE.read_bytes()
        _,cls.segments,_=elf_parts(cls.data)

    def test_valid_binding_ranges(self):
        declarations=bindings(self.data).splitlines()
        expected={pc for lo,hi in RETIRED_RANGES for pc in range(lo,hi,4)}
        self.assertEqual({int(line.split()[1],0) for line in declarations if line.startswith('stub ')},expected)
        self.assertNotIn('stub 0x141548',declarations)
        self.assertIn('native 0x141448',declarations)
        self.assertIn('native 0x141410',declarations)
        self.assertIn('native 0x141a38',declarations)
        self.assertIn('native 0x149a90',declarations)
        _,_,sections=elf_parts(self.data)
        for line in declarations:
            if line.startswith('native '):
                pc=int(line.split()[1],0)
                self.assertTrue(any(s[2]&6==6 and s[3]<=pc and pc+4<=s[3]+s[5] for s in sections))
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

    def test_game_timing_adaptations(self):
        self.assertEqual(game_adaptations(self.data).splitlines(),[
            '001002f4 ac220010 00000000','00100300 ac200000 00000000'])
        segment=next(s for s in self.segments if s[0]==1 and s[2]<=0x1002c0<0x100304<=s[2]+s[4])
        for pc in range(0x1002c0,0x100304,4):
            data=bytearray(self.data)
            data[segment[1]+pc-segment[2]]^=1
            with self.subTest(pc=hex(pc)),self.assertRaises(ValueError):
                game_adaptations(data)

    def test_worker_entry(self):
        self.assertEqual(game_entries(self.data),[
            dict(Name='worker_entry_0011d4e0',Start='0x11d4e0',End='0x11d7c0')])
        segment=next(s for s in self.segments if s[0]==1 and s[2]<=0x11d4e0<0x11d7c0<=s[2]+s[4])
        for pc in range(0x11d4e0,0x11d7c0,4):
            data=bytearray(self.data)
            data[segment[1]+pc-segment[2]]^=1
            with self.subTest(pc=hex(pc)),self.assertRaises(ValueError): game_entries(data)

    def test_boot_seed_site(self):
        self.assertEqual(native_game_sites(self.data),'native 0x1003c4\nnative 0x11fb50\n')
        segment=next(s for s in self.segments if s[0]==1 and s[2]<=0x1003bc<0x1003dc<=s[2]+s[4])
        for pc in range(0x1003bc,0x1003dc,4):
            data=bytearray(self.data);data[segment[1]+pc-segment[2]]^=1
            with self.subTest(pc=hex(pc)),self.assertRaises(ValueError):native_game_sites(data)

    def test_frame_clock_input(self):
        self.assertEqual(frame_adaptations(self.data),'0011fb54 0002137a 00000000\n')
        segment=next(s for s in self.segments if s[0]==1 and s[2]<=0x11fb40<0x11fc88<=s[2]+s[4])
        for pc in range(0x11fb40,0x11fc88,4):
            data=bytearray(self.data);data[segment[1]+pc-segment[2]]^=1
            with self.subTest(pc=hex(pc)),self.assertRaises(ValueError):frame_adaptations(data)


if __name__=='__main__': unittest.main()
