"""Private USA ELF signature tests; no game bytes are checked into this file."""
import sys
import struct
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from vp2_bozo_prepare import elf_parts,overlay_entries
from vp2_bozo_kernel import LEAFLESS_QUERY_SITES,FULL_SINE_SITES,FULL_COSINE_SITES,EULER_QUATERNION_SITES,POSITIVE_EULER_SITES,DOF_MATRIX_SITES,HERMITE_VECTOR_SITES,SQUAD_SITES,PARTICLE_PUBLICATION_SITES
from vp2_bozo_kernel import bindings, game_adaptations, frame_adaptations, game_entries, native_game_sites, GUARDS, RETIRED_RANGES,bootstrap_adaptations,bootstrap_tables,overlay_adaptations,overlay_native_sites,display_adaptations,DISPLAY_STORES,startup_clear_adaptations,composition_adaptations,composition_sites,reject_performance_counter_reads

IMAGE=Path(sys.argv.pop(1))


class KernelGuardTests(unittest.TestCase):
    def test_particle_publication_source(self):
        data=(IMAGE.parent/'mod001.elf').read_bytes();_,segments,_=elf_parts(data)
        lo,hi=0x22e650,0x22f0d4;s=next(s for s in segments if s[0]==1 and s[2]<=lo<hi<=s[2]+s[4])
        for pc in range(lo,hi,4):
            changed=bytearray(data);changed[s[1]+pc-s[2]]^=1
            with self.subTest(pc=hex(pc)),self.assertRaises(ValueError):overlay_adaptations('mod001',changed)
    def test_squad_source(self):
        data=(IMAGE.parent/'mod001.elf').read_bytes();_,segments,_=elf_parts(data)
        lo,hi=0x214c40,0x214e40;s=next(s for s in segments if s[0]==1 and s[2]<=lo<hi<=s[2]+s[4])
        actual=[pc for pc in range(lo,hi,4) if int.from_bytes(data[s[1]+pc-s[2]:s[1]+pc-s[2]+4],'little')>>26 in (0x12,0x36,0x3e)]
        self.assertEqual(list(SQUAD_SITES),sorted(actual+[0x214c40,0x214ce8]))
        for pc in range(lo,hi,4):
            changed=bytearray(data);changed[s[1]+pc-s[2]]^=1
            with self.subTest(pc=hex(pc)),self.assertRaises(ValueError):overlay_adaptations('mod001',changed)
    def test_hermite_vector_source(self):
        data=(IMAGE.parent/'mod001.elf').read_bytes();_,segments,_=elf_parts(data)
        lo,hi=0x2a68a0,0x2a691c;s=next(s for s in segments if s[0]==1 and s[2]<=lo<hi<=s[2]+s[4])
        actual=[pc for pc in range(lo,hi,4) if int.from_bytes(data[s[1]+pc-s[2]:s[1]+pc-s[2]+4],'little')>>26 in (0x12,0x36,0x3e)]
        self.assertEqual(list(HERMITE_VECTOR_SITES),actual)
        for lo,hi in ((0x2a68a0,0x2a691c),(0x2a6980,0x2a69f0)):
            s=next(s for s in segments if s[0]==1 and s[2]<=lo<hi<=s[2]+s[4])
            for pc in range(lo,hi,4):
                changed=bytearray(data);changed[s[1]+pc-s[2]]^=1
                with self.subTest(pc=hex(pc)),self.assertRaises(ValueError):overlay_adaptations('mod001',changed)
    def test_dof_matrix_source(self):
        data=(IMAGE.parent/'mod001.elf').read_bytes();_,segments,_=elf_parts(data)
        lo,hi=0x2d7a30,0x2d8ee8;s=next(s for s in segments if s[0]==1 and s[2]<=lo<hi<=s[2]+s[4])
        actual=[pc for pc in range(lo,hi,4) if int.from_bytes(data[s[1]+pc-s[2]:s[1]+pc-s[2]+4],'little')>>26 in (0x12,0x36,0x3e)]
        self.assertEqual(list(DOF_MATRIX_SITES),actual)
        for pc in range(lo,hi,4):
            changed=bytearray(data);changed[s[1]+pc-s[2]]^=1
            with self.subTest(pc=hex(pc)),self.assertRaises(ValueError):overlay_adaptations('mod001',changed)
    def test_euler_quaternion_source(self):
        data=(IMAGE.parent/'mod001.elf').read_bytes();_,segments,_=elf_parts(data)
        lo,hi=0x2162c0,0x21672c;s=next(s for s in segments if s[0]==1 and s[2]<=lo<hi<=s[2]+s[4])
        actual=[pc for pc in range(lo,hi,4) if int.from_bytes(data[s[1]+pc-s[2]:s[1]+pc-s[2]+4],'little')>>26 in (0x12,0x36,0x3e)]
        self.assertEqual(list(EULER_QUATERNION_SITES),actual)
        for pc in range(lo,hi,4):
            changed=bytearray(data);changed[s[1]+pc-s[2]]^=1
            with self.subTest(pc=hex(pc)),self.assertRaises(ValueError):overlay_adaptations('mod001',changed)
    def test_leafless_query_source(self):
        data=(IMAGE.parent/'mod001.elf').read_bytes();_,segments,_=elf_parts(data)
        lo,hi=0x1f6f40,0x1f739c;s=next(s for s in segments if s[0]==1 and s[2]<=lo<hi<=s[2]+s[4])
        actual=[pc for pc in range(lo,hi,4) if int.from_bytes(data[s[1]+pc-s[2]:s[1]+pc-s[2]+4],'little')>>26 in (0x12,0x36,0x3e)]
        self.assertEqual(list(LEAFLESS_QUERY_SITES),actual)
        for pc in range(lo,hi,4):
            changed=bytearray(data);changed[s[1]+pc-s[2]]^=1
            with self.subTest(pc=hex(pc)),self.assertRaises(ValueError):overlay_adaptations('mod001',changed)
    def test_positive_euler_source(self):
        data=(IMAGE.parent/'mod001.elf').read_bytes();_,segments,_=elf_parts(data)
        lo,hi=0x2159c0,0x215e40;s=next(s for s in segments if s[0]==1 and s[2]<=lo<hi<=s[2]+s[4])
        actual=[pc for pc in range(lo,hi,4) if int.from_bytes(data[s[1]+pc-s[2]:s[1]+pc-s[2]+4],'little')>>26 in (0x12,0x36,0x3e)]
        self.assertEqual(list(POSITIVE_EULER_SITES),actual)
        for pc in range(lo,hi,4):
            changed=bytearray(data);changed[s[1]+pc-s[2]]^=1
            with self.subTest(pc=hex(pc)),self.assertRaises(ValueError):overlay_adaptations('mod001',changed)
    def test_terrain_height_source(self):
        data=(IMAGE.parent/'mod001.elf').read_bytes();_,segments,_=elf_parts(data)
        lo,hi=0x252e30,0x253344;s=next(s for s in segments if s[0]==1 and s[2]<=lo<hi<=s[2]+s[4])
        for pc in range(lo,hi,4):
            changed=bytearray(data);changed[s[1]+pc-s[2]]^=1
            with self.subTest(pc=hex(pc)),self.assertRaises(ValueError):overlay_adaptations('mod001',changed)
    def test_height_bound_source(self):
        data=(IMAGE.parent/'mod003.elf').read_bytes();_,segments,_=elf_parts(data)
        lo,hi=0x3e5bc0,0x3e5d34;s=next(s for s in segments if s[0]==1 and s[2]<=lo<hi<=s[2]+s[4])
        for pc in range(lo,hi,4):
            changed=bytearray(data);changed[s[1]+pc-s[2]]^=1
            with self.subTest(pc=hex(pc)),self.assertRaises(ValueError):overlay_adaptations('mod003',changed)
    def test_corner_bound_source(self):
        data=(IMAGE.parent/'mod003.elf').read_bytes();_,segments,_=elf_parts(data)
        lo,hi=0x3e7e94,0x3e80f0;s=next(s for s in segments if s[0]==1 and s[2]<=lo<hi<=s[2]+s[4])
        for pc in range(lo,hi,4):
            changed=bytearray(data);changed[s[1]+pc-s[2]]^=1
            with self.subTest(pc=hex(pc)),self.assertRaises(ValueError):overlay_adaptations('mod003',changed)
    def test_record_initialization_delay(self):
        data=(IMAGE.parent/'mod003.elf').read_bytes()
        words={0x3d7718:0x3c011000,0x3d771c:0x8c360000,0x3d7740:0x3c011000,0x3d7744:0x8c220000,
               0x3d7748:0x00561023,0x3d774c:0x2c41003f,0x3d7750:0x10200005,0x3d7760:0x1000fff7}
        self.assertEqual(overlay_adaptations('mod003',data),'003d9238 ac220010 00000000\n003d95b0 ac200000 00000000\n003d5694 0c04896e 00000000\n003d5794 0c048afe 00000000\n003d57a0 0c04896e 00000000\n'+''.join(f'{pc:08x} {word:08x} 00000000\n' for pc,word in words.items())+
            '003d3648 dfbf0000 0c0f4d38\n003d364c 03e00008 00000000\n003d3650 27bd0010 dfbf0000\n003d3654 00000000 03e00008\n003d3658 00000000 27bd0010\n00447d5c 0c044c70 0c111e78\n00447a44 0c044c70 0c111e78\n')
        _,segments,_=elf_parts(data);lo,hi=0x3d7708,0x3d77cc
        s=next(s for s in segments if s[0]==1 and s[2]<=lo<hi<=s[2]+s[4])
        for pc in range(lo,hi,4):
            changed=bytearray(data);changed[s[1]+pc-s[2]]^=1
            with self.subTest(pc=hex(pc)),self.assertRaises(ValueError):overlay_adaptations('mod003',changed)
        lo,hi=0x3d5670,0x3d57ec
        s=next(s for s in segments if s[0]==1 and s[2]<=lo<hi<=s[2]+s[4])
        for pc in range(lo,hi,4):
            changed=bytearray(data);changed[s[1]+pc-s[2]]^=1
            with self.subTest(pc=hex(pc)),self.assertRaises(ValueError):overlay_adaptations('mod003',changed)

    def test_field_distance_source(self):
        data=(IMAGE.parent/'mod003.elf').read_bytes();_,segments,_=elf_parts(data)
        lo,hi=0x378908,0x3789c8;s=next(s for s in segments if s[0]==1 and s[2]<=lo<hi<=s[2]+s[4])
        for pc in range(lo,hi,4):
            changed=bytearray(data);changed[s[1]+pc-s[2]]^=1
            with self.subTest(pc=hex(pc)),self.assertRaises(ValueError):overlay_adaptations('mod003',changed)

    def test_field_frame_pacing_retirement(self):
        data=(IMAGE.parent/'mod003.elf').read_bytes();_,segments,_=elf_parts(data)
        for lo,hi in ((0x3d9230,0x3d925c),(0x3d95ac,0x3d95b8)):
            s=next(s for s in segments if s[0]==1 and s[2]<=lo<hi<=s[2]+s[4])
            for pc in range(lo,hi,4):
                changed=bytearray(data);changed[s[1]+pc-s[2]]^=1
                with self.subTest(pc=hex(pc)),self.assertRaises(ValueError):overlay_adaptations('mod003',changed)

    def test_movement_distance_source(self):
        data=(IMAGE.parent/'mod003.elf').read_bytes();_,segments,_=elf_parts(data)
        for lo,hi in ((0x3c5d08,0x3c5dc8),(0x379228,0x37928c),(0x410df0,0x410e50),(0x37b500,0x37b578),(0x381fb8,0x38204c),(0x37fa30,0x37fb44),(0x37f95c,0x37f9d8),(0x43d280,0x43d35c),(0x4126a0,0x412868)):
            s=next(s for s in segments if s[0]==1 and s[2]<=lo<hi<=s[2]+s[4])
            for pc in range(lo,hi,4):
                changed=bytearray(data);changed[s[1]+pc-s[2]]^=1
                with self.subTest(pc=hex(pc)),self.assertRaises(ValueError):overlay_adaptations('mod003',changed)

    def test_animation_movement_source(self):
        data=(IMAGE.parent/'mod003.elf').read_bytes();_,segments,_=elf_parts(data)
        lo,hi=0x446520,0x446b8c;s=next(s for s in segments if s[0]==1 and s[2]<=lo<hi<=s[2]+s[4])
        actual=[pc for pc in range(lo,hi,4) if int.from_bytes(data[s[1]+pc-s[2]:s[1]+pc-s[2]+4],'little')>>26 in (0x12,0x36,0x3e)]
        self.assertEqual(actual,list(range(0x446724,0x446748,4)))
        for pc in range(lo,hi,4):
            changed=bytearray(data);changed[s[1]+pc-s[2]]^=1
            with self.subTest(pc=hex(pc)),self.assertRaises(ValueError):overlay_adaptations('mod003',changed)

    def test_particle_motion_source(self):
        data=(IMAGE.parent/'mod003.elf').read_bytes();_,segments,_=elf_parts(data)
        lo,hi=0x408e90,0x4095d8;s=next(s for s in segments if s[0]==1 and s[2]<=lo<hi<=s[2]+s[4])
        actual=[pc for pc in range(lo,hi,4) if int.from_bytes(data[s[1]+pc-s[2]:s[1]+pc-s[2]+4],'little')>>26 in (0x12,0x36,0x3e)]
        self.assertEqual(len(actual),26)
        for pc in range(lo,hi,4):
            changed=bytearray(data);changed[s[1]+pc-s[2]]^=1
            with self.subTest(pc=hex(pc)),self.assertRaises(ValueError):overlay_adaptations('mod003',changed)

    def test_particle_physics_source(self):
        data=(IMAGE.parent/'mod003.elf').read_bytes();_,segments,_=elf_parts(data)
        lo,hi=0x40a8b0,0x40b07c;s=next(s for s in segments if s[0]==1 and s[2]<=lo<hi<=s[2]+s[4])
        for pc in range(lo,hi,4):
            changed=bytearray(data);changed[s[1]+pc-s[2]]^=1
            with self.subTest(pc=hex(pc)),self.assertRaises(ValueError):overlay_adaptations('mod003',changed)

    def test_field_debug_font_owner(self):
        data=(IMAGE.parent/'mod003.elf').read_bytes();_,segments,_=elf_parts(data)
        self.assertEqual(overlay_native_sites('mod003',data),'native 0x40b040\nnative 0x3d93f4\nnative 0x3d38d4\nnative 0x3d363c\nnative 0x3d364c\nnative 0x3d49b0\nnative 0x447af0\nnative 0x447b84\nnative 0x447d60\nnative 0x447a48\nnative 0x447f50\n'+''.join(f'native 0x{pc:x}\n' for pc in (0x3e5ca4,0x3e5ca8,0x3e5cac,0x3e5cb4,0x3e5cb8,0x3e5cc0,*range(0x3e7edc,0x3e7f38,4),*range(0x3e7fa0,0x3e7ffc,4))))
        lo,hi=0x3d93dc,0x3d9400;s=next(s for s in segments if s[0]==1 and s[2]<=lo<hi<=s[2]+s[4])
        for pc in range(lo,hi,4):
            changed=bytearray(data);changed[s[1]+pc-s[2]]^=1
            with self.subTest(pc=hex(pc)),self.assertRaises(ValueError):overlay_native_sites('mod003',changed)

    def test_flat_fade_sources(self):
        data=(IMAGE.parent/'mod003.elf').read_bytes();_,segments,_=elf_parts(data)
        self.assertIn(dict(Name='native_flat_fade_draw',Start='0x3d3620',End='0x3d3660'),overlay_entries('mod003',data))
        for lo,hi in ((0x3d3620,0x3d3660),(0x3d3660,0x3d44e4),(0x3d49b0,0x3d4af0)):
            s=next(s for s in segments if s[0]==1 and s[2]<=lo<hi<=s[2]+s[4])
            for pc in (lo,(lo+(hi-lo)//2)&~3,hi-4):
                changed=bytearray(data);changed[s[1]+pc-s[2]]^=1
                with self.subTest(pc=hex(pc)),self.assertRaises(ValueError):overlay_adaptations('mod003',changed)

    def test_fis_resource_sources(self):
        data=(IMAGE.parent/'mod003.elf').read_bytes();_,segments,_=elf_parts(data)
        for lo,hi in ((0x4479e0,0x447f50),(0x447f50,0x4480a8)):
            s=next(s for s in segments if s[0]==1 and s[2]<=lo<hi<=s[2]+s[4])
            for pc in (lo,(lo+(hi-lo)//2)&~3,hi-4):
                changed=bytearray(data);changed[s[1]+pc-s[2]]^=1
                with self.subTest(pc=hex(pc)),self.assertRaises(ValueError):overlay_adaptations('mod003',changed)

    def test_bone_matrix_sources(self):
        data=(IMAGE.parent/'mod001.elf').read_bytes();_,segments,_=elf_parts(data);lo,hi=0x2371dc,0x2372d0
        s=next(s for s in segments if s[0]==1 and s[2]<=lo<hi<=s[2]+s[4])
        for pc in (lo,0x237224,0x237284,hi-4):
            changed=bytearray(data);changed[s[1]+pc-s[2]]^=1
            with self.subTest(pc=hex(pc)),self.assertRaises(ValueError):overlay_adaptations('mod001',changed)
        for at in (0x12b4a8+0x1f0,0x12b4a8+0x248,0x12b4a8+0x8a0,0x12b4a8+0x958):
            changed=bytearray(data);changed[at]^=1
            with self.subTest(at=hex(at)),self.assertRaises(ValueError):overlay_adaptations('mod001',changed)

    def test_pair_distance_source(self):
        data=(IMAGE.parent/'mod001.elf').read_bytes();_,segments,_=elf_parts(data);lo,hi=0x1fedc0,0x1fee10
        s=next(s for s in segments if s[0]==1 and s[2]<=lo<hi<=s[2]+s[4])
        for pc in range(lo,hi,4):
            changed=bytearray(data);changed[s[1]+pc-s[2]]^=1
            with self.subTest(pc=hex(pc)),self.assertRaises(ValueError):overlay_adaptations('mod001',changed)

    def test_camera_scale_source(self):
        data=(IMAGE.parent/'mod001.elf').read_bytes();_,segments,_=elf_parts(data);lo,hi=0x20f240,0x20f2bc
        s=next(s for s in segments if s[0]==1 and s[2]<=lo<hi<=s[2]+s[4])
        for pc in range(lo,hi,4):
            changed=bytearray(data);changed[s[1]+pc-s[2]]^=1
            with self.subTest(pc=hex(pc)),self.assertRaises(ValueError):overlay_adaptations('mod001',changed)

    def test_model_parameter_source(self):
        data=(IMAGE.parent/'mod001.elf').read_bytes();_,segments,_=elf_parts(data);lo,hi=0x23c240,0x23c2d0
        s=next(s for s in segments if s[0]==1 and s[2]<=lo<hi<=s[2]+s[4])
        for pc in range(lo,hi,4):
            changed=bytearray(data);changed[s[1]+pc-s[2]]^=1
            with self.subTest(pc=hex(pc)),self.assertRaises(ValueError):overlay_adaptations('mod001',changed)

    def test_inverse_selector_source(self):
        data=(IMAGE.parent/'mod001.elf').read_bytes();_,segments,_=elf_parts(data);lo,hi=0x248560,0x248630
        s=next(s for s in segments if s[0]==1 and s[2]<=lo<hi<=s[2]+s[4])
        for pc in range(lo,hi,4):
            changed=bytearray(data);changed[s[1]+pc-s[2]]^=1
            with self.subTest(pc=hex(pc)),self.assertRaises(ValueError):overlay_adaptations('mod001',changed)
        for at in (0x12b4a8+0x7f0,0x12b4a8+0x800,0x12b4a8+0x834):
            changed=bytearray(data);changed[at]^=1
            with self.subTest(at=hex(at)),self.assertRaises(ValueError):overlay_adaptations('mod001',changed)

    def test_column_inverse_source(self):
        data=(IMAGE.parent/'mod001.elf').read_bytes();_,segments,_=elf_parts(data);lo,hi=0x216cb0,0x216d60
        s=next(s for s in segments if s[0]==1 and s[2]<=lo<hi<=s[2]+s[4])
        for pc in range(lo,hi,4):
            changed=bytearray(data);changed[s[1]+pc-s[2]]^=1
            with self.subTest(pc=hex(pc)),self.assertRaises(ValueError):overlay_adaptations('mod001',changed)

    def test_transposed_inverse_and_constraint_caller_sources(self):
        data=(IMAGE.parent/'mod001.elf').read_bytes();_,segments,_=elf_parts(data)
        for lo,hi in ((0x216d60,0x216e40),(0x1fcb40,0x1fd110)):
            s=next(s for s in segments if s[0]==1 and s[2]<=lo<hi<=s[2]+s[4])
            for pc in range(lo,hi,4):
                changed=bytearray(data);changed[s[1]+pc-s[2]]^=1
                with self.subTest(pc=hex(pc)),self.assertRaises(ValueError):overlay_adaptations('mod001',changed)

    def test_inverse_cosine_source(self):
        data=(IMAGE.parent/'mod001.elf').read_bytes();_,segments,_=elf_parts(data);lo,hi=0x214e40,0x2150f0
        s=next(s for s in segments if s[0]==1 and s[2]<=lo<hi<=s[2]+s[4])
        for pc in range(lo,hi,4):
            changed=bytearray(data);changed[s[1]+pc-s[2]]^=1
            with self.subTest(pc=hex(pc)),self.assertRaises(ValueError):overlay_adaptations('mod001',changed)

    def test_quaternion_interpolation_source(self):
        data=(IMAGE.parent/'mod001.elf').read_bytes();_,segments,_=elf_parts(data);lo,hi=0x2151d0,0x2153a0
        s=next(s for s in segments if s[0]==1 and s[2]<=lo<hi<=s[2]+s[4])
        for pc in range(lo,hi,4):
            changed=bytearray(data);changed[s[1]+pc-s[2]]^=1
            with self.subTest(pc=hex(pc)),self.assertRaises(ValueError):overlay_adaptations('mod001',changed)

    def test_three_sine_source(self):
        data=(IMAGE.parent/'mod001.elf').read_bytes();_,segments,_=elf_parts(data);lo,hi=0x2150f0,0x2151d0
        s=next(s for s in segments if s[0]==1 and s[2]<=lo<hi<=s[2]+s[4])
        for pc in range(lo,hi,4):
            changed=bytearray(data);changed[s[1]+pc-s[2]]^=1
            with self.subTest(pc=hex(pc)),self.assertRaises(ValueError):overlay_adaptations('mod001',changed)
        for at in range(0x12b4a8+0x9f0,0x12b4a8+0xaa8,4):
            changed=bytearray(data);changed[at]^=1
            with self.subTest(at=hex(at)),self.assertRaises(ValueError):overlay_adaptations('mod001',changed)

    def test_velocity_loop_source(self):
        data=(IMAGE.parent/'mod001.elf').read_bytes();_,segments,_=elf_parts(data);lo,hi=0x1fe8a8,0x1fe95c
        s=next(s for s in segments if s[0]==1 and s[2]<=lo<hi<=s[2]+s[4])
        for pc in range(lo,hi,4):
            changed=bytearray(data);changed[s[1]+pc-s[2]]^=1
            with self.subTest(pc=hex(pc)),self.assertRaises(ValueError):overlay_adaptations('mod001',changed)

    def test_point_direction_source(self):
        data=(IMAGE.parent/'mod001.elf').read_bytes();_,segments,_=elf_parts(data);lo,hi=0x1fb6a0,0x1fb7b8
        s=next(s for s in segments if s[0]==1 and s[2]<=lo<hi<=s[2]+s[4])
        for pc in range(lo,hi,4):
            changed=bytearray(data);changed[s[1]+pc-s[2]]^=1
            with self.subTest(pc=hex(pc)),self.assertRaises(ValueError):overlay_adaptations('mod001',changed)

    def test_full_transform_sources(self):
        data=(IMAGE.parent/'mod001.elf').read_bytes();_,segments,_=elf_parts(data)
        for lo,hi in ((0x1fb3f4,0x1fb41c),(0x1fb5ec,0x1fb614),(0x1fb87c,0x1fb8a4),
                      (0x1fbdcc,0x1fbdf4),(0x1fbe60,0x1fbfa8)):
            s=next(s for s in segments if s[0]==1 and s[2]<=lo<hi<=s[2]+s[4])
            for pc in range(lo,hi,4):
                changed=bytearray(data);changed[s[1]+pc-s[2]]^=1
                with self.subTest(pc=hex(pc)),self.assertRaises(ValueError):overlay_adaptations('mod001',changed)

    def test_motion_source(self):
        for name,lo,hi in (('mod001',0x1fc9e0,0x1fcb3c),('mod001',0x1fc860,0x1fc9e0),('mod001',0x1fc6d0,0x1fc860),('mod001',0x1fde10,0x1fde88),('mod001',0x2120a0,0x212950),('mod001',0x1fc3dc,0x1fc488),('mod001',0x1fc5b0,0x1fc6d0),('mod001',0x1fc030,0x1fc180),('mod001',0x1fde88,0x1fe024),('mod001',0x211fb0,0x2120a0),('mod001',0x1fc244,0x1fc2f8),('mod001',0x1fc4f0,0x1fc5b0),('mod001',0x1fbfb0,0x1fc030),('mod001',0x1fe1e0,0x1fe258),('mod003',0x3749f0,0x374a04)):
            data=(IMAGE.parent/(name+'.elf')).read_bytes();_,segments,_=elf_parts(data)
            s=next(s for s in segments if s[0]==1 and s[2]<=lo<hi<=s[2]+s[4])
            for pc in range(lo,hi,4):
                changed=bytearray(data);changed[s[1]+pc-s[2]]^=1
                with self.subTest(pc=hex(pc)),self.assertRaises(ValueError):overlay_adaptations(name,changed)

    def test_linear_transform_sources(self):
        data=(IMAGE.parent/'mod001.elf').read_bytes();_,segments,_=elf_parts(data)
        for lo,hi in ((0x1fb1e4,0x1fb204),(0x1fb2fc,0x1fb31c),(0x1fb42c,0x1fb44c),(0x1fb624,0x1fb644)):
            s=next(s for s in segments if s[0]==1 and s[2]<=lo<hi<=s[2]+s[4])
            for pc in range(lo,hi,4):
                changed=bytearray(data);changed[s[1]+pc-s[2]]^=1
                with self.subTest(pc=hex(pc)),self.assertRaises(ValueError):overlay_adaptations('mod001',changed)

    def test_angle_visibility_program(self):
        data=(IMAGE.parent/'mod001.elf').read_bytes()
        for at in (0x12b4a8+0xc90,0x12b4a8+0xd20,0x12b4a8+0xd5c):
            changed=bytearray(data);changed[at]^=1
            with self.subTest(at=hex(at)),self.assertRaises(ValueError):overlay_adaptations('mod001',changed)
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
        entries=overlay_entries('mod001',data);self.assertEqual(len(entries),54)
        self.assertIn(dict(Name='native_empty_child_draw',Start='0x2075a0',End='0x2075e0'),entries)
        self.assertIn(dict(Name='ui_transition_002ea350',Start='0x2ea350',End='0x2ea6f0'),entries)
        self.assertIn(dict(Name='camera_method_001e8080',Start='0x1e8080',End='0x1e80a0'),entries)
        self.assertEqual(entries[0],dict(Name='module_initializer_00358200',Start='0x358200',End='0x358230'))
        _,segments,_=elf_parts(data)
        for lo,hi in ((0x2075a0,0x2075e0),(0x358200,0x358a10),(0x358a10,0x358a7c),(0x29e240,0x29e280),
                      (0x1e7fd0,0x1e8220),(0x1fae30,0x1faeb0),(0x2ea350,0x2ea6f0),(0x357960,0x357984)):
            segment=next(s for s in segments if s[0]==1 and s[2]<=lo<hi<=s[2]+s[4])
            for pc in range(lo,hi,4):
                altered=bytearray(data);altered[segment[1]+pc-segment[2]]^=1
                with self.subTest(pc=hex(pc)),self.assertRaises(ValueError):overlay_entries('mod001',altered)
        second=(IMAGE.parent/'mod002.elf').read_bytes()
        self.assertEqual(len(overlay_entries('mod002',second)),2)

    def test_graphics_enable_retirement(self):
        data=(IMAGE.parent/'mod001.elf').read_bytes()
        self.assertEqual(overlay_adaptations('mod001',data).splitlines(),[
            '0030ce84 3c011000 00000000','0030ce8c 8c230000 00000000','0030ce90 00621021 00000000',
            '0030ce94 3042000f 00000000','0030ce98 00021100 00000000','0030ce9c 02021021 00000000',
            '00306a10 3c011000 00000000','00306a14 00102402 00000000','00306a18 8c230000 00000000','00306a1c 00823021 00000000',
            '00306a28 000310c0 00000000','00306a2c 00431023 00000000','00306a30 00468821 00000000','00306a38 02338821 00000000',
            '00306b84 3c011000 00000000','00306b8c 8c340000 00000000','00306bb8 3c011000 00000000','00306bbc 8c220000 00000000',
            '00306bc0 00541023 00000000','00306bc4 2c41002b 00000000','00306bcc 3c011000 00000000','00306bd4 8c340000 00000000',
            '00249830 ac22e000 00000000',
            '002075c8 dfbf0000 0c081d60','002075cc 03e00008 00000000','002075d0 27bd0010 dfbf0000',
            '002075d4 00000000 03e00008','002075d8 00000000 27bd0010',
            '00221300 0c0a75cc 00000000','00221554 4100fffa 00000000','00221568 0c04df7e 00000000',
            '002216fc 0c0a75ac 00000000','0022172c 4100fffa 00000000','00221740 0c04df7e 00000000',
            '002088ec 0c0807c8 00000000',
            '002cb48c 0c0807c8 0c087268',
            '00239fec 0c0807c8 0c087268',
            '002dbdcc 0c0807c8 0c0b6f64',
            '002d934c 0c0807c8 0c0b64c4',
            '0022ee50 1a600009 10000009',
            '0022f048 1a600009 10000009',
            '0025aa2c 0c0807c8 0c087268',
            '002f3710 0c0bbce4 0c087268',
            '002ec9e0 0c0bbce4 0c087268',
            '002bc218 afa2003c 080af0b4','002bc120 3c011001 080af074','002bc124 ac20b420 00000000',
            '001ef864 0c07bd8a 00000000','001ef86c 0c07c10e 00000000',
            '001efa40 0c0504a8 0807beb8','001efae8 ac430000 03e00008','001efaec 0000282d 27bd0010',
            '001f048c 8ca30000 00000000','001f049c 00641824 00000000','001f04a4 00661825 00000000','001f04a8 aca30000 00000000',
            '002021c4 3c011001 0808087a','002021c8 2402fffd 00000000',
            '0029e210 4a00fc3e 00000000','0029e214 4b1f043d 00000000','0029e2fc 0c0807c8 00000000',
            '0029e1a0 a0820050 00000000','0029e1ac 0c0807c8 00000000',
            '0029e160 a0820050 00000000','0029e16c 0c0807c8 00000000',
            '002bef34 4080c000 00000000','002bef38 4080c004 00000000',
            '002bef3c 4080c005 00000000','002bef40 4080c006 00000000','002bef44 4080c007 00000000',
            '00208a90 0c0807c8 00000000',
            '0029cd7c 00a0302d 0220202d','0029cd80 0c044c70 0c0402cc',
            '0029cf14 00a0302d 0260202d','0029cf18 0c044c70 0c0402cc',
            '0029cc00 00a0302d 0220202d','0029cc04 0c044c70 0c0402cc',
            '002c3cb4 8c220000 00000000','002c3cb8 10000015 080b0f48'])
        self.assertEqual(overlay_native_sites('mod001',data),''.join(f'native 0x{pc:x}\n' for pc in (
        *LEAFLESS_QUERY_SITES,
        *FULL_SINE_SITES,
        *FULL_COSINE_SITES,
        *EULER_QUATERNION_SITES,
        *POSITIVE_EULER_SITES,
        *DOF_MATRIX_SITES,
        *HERMITE_VECTOR_SITES,
        *SQUAD_SITES,
        *PARTICLE_PUBLICATION_SITES,
        0x2dbde0,0x2dbeb4,0x2dc6c8,0x2dbdd0,0x2dc700,
        0x2d9360,0x2cd020,0x2d18a0,0x2da980,0x2d9350,0x2da9b0,
        *range(0x252ec8,0x252f24,4),0x252fdc,0x252fe0,0x252fe4,0x252fec,0x252ff0,0x252ff8,
        0x2bf040,0x2bf058,
        0x2371e8,0x2371ec,0x2371f0,0x2371f4,0x2371fc,0x237200,0x237204,0x237208,
        0x23720c,0x237210,0x237214,0x237218,0x237224,0x237260,0x237264,0x237268,0x23726c,
        0x237270,0x237274,0x237278,0x23727c,0x237280,0x237284,0x237288,0x23728c,0x237290,
        0x23729c,0x2372b8,0x2372bc,0x2372c0,0x2372c4,0x2372c8,
        0x2075bc,0x2075cc,0x30ce84,0x306a10,0x306b84,0x306bc4,0x306c58,
        0x30ee00,0x30ef20,0x30ef2c,0x30ef44,0x30ef78,0x30efb8,0x30eff4,0x30f028,0x30f070,0x30ed90,
        0x214540,0x2e9640,0x2e970c,0x2e95d0,0x2e9e90,0x2ea154,0x2ea338,0x2ead10,0x2eae90,0x2eb07c,0x2eb284,0x2eb2ec,0x2eb300,0x2eb320,0x2eb340,0x2eb378,
            0x29e1a0,0x2c3cb4,0x29cc30,0x29cd7c,0x29cf14,0x29cf30,0x29cc00,0x29e160,
            0x208a94,0x29af80,0x29b2a8,0x29caf8,0x2205c0,0x220ce0,0x29e43c,0x29e444,
            0x22106c,0x221294,0x221308,0x221540,0x221704,0x221718,0x221748,0x2088f0,
            0x221698,0x22169c,0x21c9dc,0x21c9e0,0x21c9e8,0x21c9ec,0x21ca1c,0x21ca20,0x21ca24,0x21ca28,0x2cb490,0x2f3714,0x2ea9a0,
            0x2e74f0,0x2e7520,0x2e7550,0x2e757c,0x2e75a0,0x2f0de0,0x2e83e4,0x2ec9e4,0x2ec308,0x2ec320,
            0x2efe40,0x2effe8,0x2eb3a0,0x2eb5c4,0x2ebac4,0x2ebb0c,0x2ebb58,0x2ebb9c,0x2ebba8,0x2bbfc0,0x2bc1d0,0x2bc1e0,0x2ba670,0x1ef6f8,0x1ef914,0x1f0438,0x1f04a0,0x1efa30,
            0x1f4590,0x1f4940,0x2bdac0,0x2bd9b0,0x2bd5d8,0x2be3d0,0x2bea60,0x239f70,0x239ff0,0x25aa30,0x25aa40,0x25bad0,0x25bdcc,0x25adbc,0x214388,0x2f2df0,0x2f2fb8,0x2f2d70))+'stub 0x2b90a0\nstub 0x2b9560\nstub 0x1f04c8\n')
        _,segments,_=elf_parts(data)
        for lo,hi in ((0x249720,0x24a438),(0x202150,0x202200),(0x29e1d0,0x29e22c),
                      (0x29e230,0x29e310),(0x333640,0x333650),(0x333650,0x333d60),
                      (0x29e0b0,0x29e1c4),(0x312280,0x313100),(0x314138,0x31413c),
                      (0x217550,0x2175c0),(0x215870,0x2159c0),(0x215640,0x2156c8),(0x2076a0,0x2077e0),(0x2075a0,0x2075e0),(0x216730,0x216780),(0x2494c0,0x249720),(0x249270,0x249390),(0x211da0,0x211dc0),(0x2c3c10,0x2c4378),(0x29cc30,0x29cf58),
                      (0x29cb20,0x29cc30),(0x2b8f90,0x2b8fb4),(0x314200,0x314600),(0x208900,0x208aac),(0x2092f0,0x20975c),(0x2097c0,0x209818),(0x2bef20,0x2bf0dc),(0x248fc0,0x24926c),(0x213fd0,0x2140f4),(0x358490,0x358690),(0x216e40,0x216f4c),(0x213b50,0x213e9c),(0x216960,0x216a60),
                      (0x208cd0,0x2091d8),(0x207d70,0x2081c8),(0x208360,0x208470),(0x20f440,0x20f938),(0x207cc0,0x207d70),(0x2205c0,0x220d14),
                      (0x29e310,0x29e378),(0x29e420,0x29e4a4),(0x221030,0x2210b8),(0x221260,0x22177c),
                      (0x29d730,0x29d7ac),(0x29d6b0,0x29d72c),(0x29df60,0x29e06c),(0x2088d0,0x208900),(0x208870,0x2088c0),(0x21c9a0,0x21ca50),
                      (0x2cb470,0x2cb4a0),(0x2cb400,0x2cb450),(0x2cb640,0x2cb8d0),
                      (0x2f36f0,0x2f3724),(0x2f37b0,0x2f38f0),(0x2ea9a0,0x2eaa10),(0x2210c0,0x221260),(0x3140e0,0x314120),
                      (0x2e74f0,0x2e75bc),(0x2f0de0,0x2f0e70),(0x2e7e90,0x2e8678),(0x2ec9c0,0x2eca00),
                      (0x2ec260,0x2ec320),(0x2ec320,0x2ec390),(0x2ec390,0x2ec610),(0x2efe40,0x2f0010),(0x2eb3a0,0x2ebbe0),(0x2bbfc0,0x2bc1dc),(0x2bc1e0,0x2bc31c),(0x2bc330,0x2bc3f4),(0x2ba600,0x2ba6d0),(0x1f02a0,0x1f0350),(0x2be600,0x2be940),
                      (0x1ef6f8,0x1ef930),(0x1f06c8,0x1f071c),(0x1f08e8,0x1f090c),(0x1f0920,0x1f098c),(0x2ba4a0,0x2ba5f8),(0x1f0510,0x1f0544),(0x1f4990,0x1f4ac0),(0x2ba3b0,0x2ba3d4),(0x313f00,0x313fa0),(0x1f0438,0x1f04b4),(0x1efa30,0x1efaf8),
                      (0x1f1f70,0x1f2140),(0x1f2140,0x1f27e8),(0x1f27e8,0x1f4590),(0x1f4590,0x1f4970),(0x2bdf48,0x2bdf80),(0x356270,0x356280),(0x2b90a0,0x2b92e0),(0x2bd9b0,0x2bdc20),(0x2b9f00,0x2ba040),
                      (0x2b9560,0x2b9b80),(0x2be020,0x2be600),(0x2bd400,0x2bd9b0),(0x2bea60,0x2beae0),(0x2bc1e0,0x2bc330),(0x1f04b8,0x1f0510),(0x239ce0,0x239e00),(0x312d50,0x312d98),(0x312e10,0x312f38),(0x23c17c,0x23c18c),(0x239f70,0x239fa8)):
            segment=next(s for s in segments if s[0]==1 and s[2]<=lo<hi<=s[2]+s[4])
            for pc in range(lo,hi,4):
                altered=bytearray(data);altered[segment[1]+pc-segment[2]]^=1
                with self.subTest(pc=hex(pc)),self.assertRaises(ValueError):overlay_adaptations('mod001',altered)

    def test_menu_hblank_retirement(self):
        data=(IMAGE.parent/'mod002.elf').read_bytes()
        self.assertEqual(overlay_adaptations('mod002',data).splitlines(),[
            '0036224c ac220010 00000000','00362760 ac200000 00000000','00360ef4 0c048afe 00000000','003721ac 03e00008 08087268'])
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
        for lo,hi in ((0x372180,0x3721b4),(0x372060,0x372180),(0x3721c0,0x37293c),(0x372040,0x37205c)):
            s=next(s for s in segments if s[0]==1 and s[2]<=lo<hi<=s[2]+s[4])
            for pc in range(lo,hi,4):
                altered=bytearray(data);altered[s[1]+pc-s[2]]^=1
                with self.subTest(pc=hex(pc)),self.assertRaises(ValueError):overlay_adaptations('mod002',altered)

    def test_native_model_filter_source_mutations(self):
        data=(IMAGE.parent/'mod001.elf').read_bytes();_,segments,_=elf_parts(data)
        for lo,hi in ((0x239fd0,0x23a000),(0x25aa10,0x25aa40),(0x25aa40,0x25be00),(0x214300,0x2143d4),(0x2143e0,0x2144bc),(0x214210,0x2142f4),(0x2f2d70,0x2f2de0),(0x2f2df0,0x2f2fd8),(0x2dbdb0,0x2dbde0),(0x2dbde0,0x2dc6f4),(0x2dc850,0x2dc96c),(0x2dc990,0x2dc9a8),(0x2dbd90,0x2dbdac),(0x2c81d0,0x2c8320)):
            s=next(s for s in segments if s[0]==1 and s[2]<=lo<hi<=s[2]+s[4])
            for pc in range(lo,hi,4):
                changed=bytearray(data);changed[s[1]+pc-s[2]]^=1
                with self.subTest(pc=hex(pc)),self.assertRaises(ValueError):overlay_adaptations('mod001',changed)

    def test_frame_copy_writer_guards(self):
        data=(IMAGE.parent/'mod001.elf').read_bytes();_,segments,_=elf_parts(data)
        for lo,hi in ((0x2c76d0,0x2c81c4),(0x2dc700,0x2dc848)):
            s=next(s for s in segments if s[0]==1 and s[2]<=lo<hi<=s[2]+s[4])
            for pc in range(lo,hi,4):
                changed=bytearray(data);changed[s[1]+pc-s[2]]^=1
                with self.subTest(pc=hex(pc)),self.assertRaises(ValueError):overlay_adaptations('mod001',changed)

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
            dict(Name='object_accessor_001010b0',Start='0x1010b0',End='0x1010bc'),
            dict(Name='bounded_string_compare_00134b60',Start='0x134b60',End='0x134d18')])
        segment=next(s for s in self.segments if s[0]==1 and s[2]<=0x11d4e0<0x11d7c0<=s[2]+s[4])
        for pc in range(0x11d4e0,0x11d7c0,4):
            data=bytearray(self.data)
            data[segment[1]+pc-segment[2]]^=1
            with self.subTest(pc=hex(pc)),self.assertRaises(ValueError): game_entries(data)
        for lo,hi in ((0x107e10,0x107e30),(0x1013d0,0x1013f0),(0x1010a0,0x1010e0),(0x154f90,0x155030),(0x134b60,0x134d18)):
            segment=next(s for s in self.segments if s[0]==1 and s[2]<=lo<hi<=s[2]+s[4])
            for pc in range(lo,hi,4):
                data=bytearray(self.data);data[segment[1]+pc-segment[2]]^=1
                with self.subTest(pc=hex(pc)),self.assertRaises(ValueError):game_entries(data)

    def test_boot_seed_site(self):
        self.assertEqual(native_game_sites(self.data),'native 0x1003c4\nnative 0x11fb50\nnative 0x11f62c\nnative 0x1131c0\nnative 0x102650\nnative 0x102998\nnative 0x105a30\nnative 0x105b58\nnative 0x105940\nnative 0x105a10\nnative 0x118230\n'+composition_sites()+'native 0x120f4c\nnative 0x120f58\n')
        segment=next(s for s in self.segments if s[0]==1 and s[2]<=0x1003bc<0x1003dc<=s[2]+s[4])
        for pc in range(0x1003bc,0x1003dc,4):
            data=bytearray(self.data);data[segment[1]+pc-segment[2]]^=1
            with self.subTest(pc=hex(pc)),self.assertRaises(ValueError):native_game_sites(data)

    def test_unused_performance_counter_admission(self):
        reject_performance_counter_reads(self.data)
        for name in ('mod001','mod002'):
            data=(IMAGE.parent/f'{name}.elf').read_bytes();reject_performance_counter_reads(data)
            _,_,sections=elf_parts(data);section=next(s for s in sections if s[2]&6==6)
            for register in (24,25):
                changed=bytearray(data);struct.pack_into('<I',changed,section[4],0x40020000 | register<<11)
                with self.assertRaises(ValueError):reject_performance_counter_reads(changed)

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

    def test_frame_composition(self):
        patches=composition_adaptations(self.data).splitlines()
        self.assertEqual({int(line.split()[0],16) for line in patches},{0x1145f4,0x114630,0x114710,0x114a74,0x114770,0x1147a8,0x114ab4,0x114ac0})
        for lo,hi in ((0x114550,0x114dd0),(0x1166e0,0x117120),(0x1178f0,0x117b00)):
            s=next(s for s in self.segments if s[0]==1 and s[2]<=lo<hi<=s[2]+s[4])
            for pc in (lo,(lo+(hi-lo)//2)&~3,hi-4):
                data=bytearray(self.data);data[s[1]+pc-s[2]]^=1
                with self.subTest(pc=hex(pc)),self.assertRaises(ValueError):composition_adaptations(data)


if __name__=='__main__': unittest.main()
