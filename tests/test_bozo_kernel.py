"""Private USA ELF signature tests; no game bytes are checked into this file."""
import sys
import struct
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from vp2_bozo_prepare import elf_parts,overlay_entries
from vp2_bozo_kernel import bindings, game_adaptations, frame_adaptations, game_entries, native_game_sites, GUARDS, RETIRED_RANGES,bootstrap_adaptations,bootstrap_tables,overlay_adaptations,overlay_native_sites,display_adaptations,DISPLAY_STORES,startup_clear_adaptations

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

    def test_display_selection_retirement(self):
        lines=display_adaptations(self.data).splitlines()
        self.assertEqual([int(line.split()[0],16) for line in lines],list(DISPLAY_STORES))
        self.assertTrue(all(line.endswith('00000000') for line in lines))
        lo,hi=0x114240,0x114544;s=next(s for s in self.segments if s[0]==1 and s[2]<=lo<hi<=s[2]+s[4])
        for pc in range(lo,hi,4):
            changed=bytearray(self.data);changed[s[1]+pc-s[2]]^=1
            with self.subTest(pc=hex(pc)),self.assertRaises(ValueError):display_adaptations(changed)

    def test_startup_physical_clear_retirement(self):
        self.assertEqual(startup_clear_adaptations(self.data),'00100708 0c048afe 00000000\n')
        lo,hi=0x100694,0x100744;s=next(s for s in self.segments if s[0]==1 and s[2]<=lo<hi<=s[2]+s[4])
        for pc in range(lo,hi,4):
            changed=bytearray(self.data);changed[s[1]+pc-s[2]]^=1
            with self.subTest(pc=hex(pc)),self.assertRaises(ValueError):startup_clear_adaptations(changed)

    def test_native_bootstrap_guards(self):
        patches=bootstrap_adaptations(self.data).splitlines()
        self.assertEqual(len(patches),7)
        self.assertEqual(bootstrap_tables(self.data)[:8],b'VPBK'+struct.pack('<I',32))
        for lo,hi in ((0x10087c,0x100930),(0x145d98,0x145df0)):
            segment=next(s for s in self.segments if s[0]==1 and s[2]<=lo<hi<=s[2]+s[4])
            for pc in range(lo,hi,4):
                data=bytearray(self.data);data[segment[1]+pc-segment[2]]^=1
                with self.subTest(pc=hex(pc)),self.assertRaises(ValueError):bootstrap_adaptations(data)

    def test_overlay_initializer(self):
        data=(IMAGE.parent/'mod001.elf').read_bytes()
        entries=overlay_entries('mod001',data);self.assertEqual(len(entries),52)
        self.assertIn(dict(Name='camera_method_001e8080',Start='0x1e8080',End='0x1e80a0'),entries)
        self.assertEqual(entries[0],dict(Name='module_initializer_00358200',Start='0x358200',End='0x358230'))
        _,segments,_=elf_parts(data)
        for lo,hi in ((0x358200,0x358a10),(0x358a10,0x358a7c),(0x29e240,0x29e280),
                      (0x1e7fd0,0x1e8220),(0x1fae30,0x1faeb0)):
            segment=next(s for s in segments if s[0]==1 and s[2]<=lo<hi<=s[2]+s[4])
            for pc in range(lo,hi,4):
                altered=bytearray(data);altered[segment[1]+pc-segment[2]]^=1
                with self.subTest(pc=hex(pc)),self.assertRaises(ValueError):overlay_entries('mod001',altered)
        second=(IMAGE.parent/'mod002.elf').read_bytes()
        self.assertEqual(len(overlay_entries('mod002',second)),2)

    def test_graphics_enable_retirement(self):
        data=(IMAGE.parent/'mod001.elf').read_bytes()
        self.assertEqual(overlay_adaptations('mod001',data).splitlines(),[
            '00249830 ac22e000 00000000','002021c4 3c011001 0808087a','002021c8 2402fffd 00000000',
            '0029e210 4a00fc3e 00000000','0029e214 4b1f043d 00000000','0029e2fc 0c0807c8 00000000',
            '0029e1a0 a0820050 00000000','0029e1ac 0c0807c8 00000000',
            '0029cd7c 00a0302d 0220202d','0029cd80 0c044c70 0c0402cc',
            '0029cf14 00a0302d 0260202d','0029cf18 0c044c70 0c0402cc',
            '0029cc00 00a0302d 0220202d','0029cc04 0c044c70 0c0402cc',
            '002c3cb4 8c220000 00000000','002c3cb8 10000015 080b0f48'])
        self.assertEqual(overlay_native_sites('mod001',data),'native 0x29e1a0\nnative 0x2c3cb4\nnative 0x29cc30\nnative 0x29cd7c\nnative 0x29cf14\nnative 0x29cf30\nnative 0x29cc00\n')
        _,segments,_=elf_parts(data)
        for lo,hi in ((0x249720,0x24a438),(0x202150,0x202200),(0x29e1d0,0x29e22c),
                      (0x29e230,0x29e310),(0x333640,0x333650),(0x333650,0x333d60),
                      (0x29e0b0,0x29e1c4),(0x312280,0x313100),(0x314138,0x31413c),
                      (0x217550,0x2175c0),(0x216730,0x216780),(0x2494c0,0x249720),(0x249270,0x249390),(0x211da0,0x211dc0),(0x2c3c10,0x2c4378),(0x29cc30,0x29cf58),
                      (0x29cb20,0x29cc30),(0x2b8f90,0x2b8fb4),(0x314200,0x314600)):
            segment=next(s for s in segments if s[0]==1 and s[2]<=lo<hi<=s[2]+s[4])
            for pc in range(lo,hi,4):
                altered=bytearray(data);altered[segment[1]+pc-segment[2]]^=1
                with self.subTest(pc=hex(pc)),self.assertRaises(ValueError):overlay_adaptations('mod001',altered)

    def test_menu_hblank_retirement(self):
        data=(IMAGE.parent/'mod002.elf').read_bytes()
        self.assertEqual(overlay_adaptations('mod002',data).splitlines(),[
            '0036224c ac220010 00000000','00362760 ac200000 00000000','00360ef4 0c048afe 00000000'])
        _,segments,_=elf_parts(data);lo,hi=0x3617f0,0x362764
        s=next(s for s in segments if s[0]==1 and s[2]<=lo<hi<=s[2]+s[4])
        for pc in range(lo,hi,4):
            altered=bytearray(data);altered[s[1]+pc-s[2]]^=1
            with self.subTest(pc=hex(pc)),self.assertRaises(ValueError):overlay_adaptations('mod002',altered)
        lo,hi=0x360dd0,0x360f44
        s=next(s for s in segments if s[0]==1 and s[2]<=lo<hi<=s[2]+s[4])
        for pc in range(lo,hi,4):
            altered=bytearray(data);altered[s[1]+pc-s[2]]^=1
            with self.subTest(pc=hex(pc)),self.assertRaises(ValueError):overlay_adaptations('mod002',altered)

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
            dict(Name='worker_entry_0011d4e0',Start='0x11d4e0',End='0x11d7c0'),
            dict(Name='thread_adjustor_00107e10',Start='0x107e10',End='0x107e18'),
            dict(Name='thread_adjustor_00107e20',Start='0x107e20',End='0x107e28'),
            dict(Name='object_constructor_001013d0',Start='0x1013d0',End='0x1013ec'),
            dict(Name='object_accessor_001010a0',Start='0x1010a0',End='0x1010ac'),
            dict(Name='object_accessor_001010b0',Start='0x1010b0',End='0x1010bc')])
        segment=next(s for s in self.segments if s[0]==1 and s[2]<=0x11d4e0<0x11d7c0<=s[2]+s[4])
        for pc in range(0x11d4e0,0x11d7c0,4):
            data=bytearray(self.data)
            data[segment[1]+pc-segment[2]]^=1
            with self.subTest(pc=hex(pc)),self.assertRaises(ValueError): game_entries(data)
        for lo,hi in ((0x107e10,0x107e30),(0x1013d0,0x1013f0),(0x1010a0,0x1010e0),(0x154f90,0x155030)):
            segment=next(s for s in self.segments if s[0]==1 and s[2]<=lo<hi<=s[2]+s[4])
            for pc in range(lo,hi,4):
                data=bytearray(self.data);data[segment[1]+pc-segment[2]]^=1
                with self.subTest(pc=hex(pc)),self.assertRaises(ValueError):game_entries(data)

    def test_boot_seed_site(self):
        self.assertEqual(native_game_sites(self.data),'native 0x1003c4\nnative 0x11fb50\nnative 0x11f62c\n')
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
        lo,hi=0x11f5a0,0x11f638
        segment=next(s for s in self.segments if s[0]==1 and s[2]<=lo<hi<=s[2]+s[4])
        for pc in range(lo,hi,4):
            data=bytearray(self.data);data[segment[1]+pc-segment[2]]^=1
            with self.subTest(pc=hex(pc)),self.assertRaises(ValueError):native_game_sites(data)


if __name__=='__main__': unittest.main()
