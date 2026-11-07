"""Semantic conversion tests, using invented mesh records only."""
import struct
import sys
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from vp2_bozo_meshes import convert


def pack(material=True,algorithms=(0,0,0,0)):
    out=bytearray(struct.pack('<4sI4Ii8i3fI20I',b'VPM3',1,96,3,1,0,2,
                             *algorithms,0,0,0,0,1,2,3,int(material),*([0]*20)))
    if material:out+=struct.pack('<16I',0x48,3,0,0x1c,0x70,0,*([0]*10))
    out+=struct.pack('<2H',3,0)
    out+=struct.pack('<30I',*range(30))
    if 4 in algorithms:out+=struct.pack('<12I',*range(12))
    if 48 in algorithms:out+=struct.pack('<i',2)
    out+=struct.pack('<I6I',256,0,12,24,36,0xffffffff,60)
    return out


class MeshConversionTests(unittest.TestCase):
    def test_semantic_material(self):
        result=convert(pack());header=struct.unpack_from('<4sI4I4i12I',result)
        self.assertEqual(header[:6],(b'VPN1',1,96,3,1,2))
        self.assertEqual(header[10:],(1,1,1,1,3,3,1,1,0,2,0,1))
        self.assertEqual(struct.unpack_from('<H',result,88),(3,))

    def test_algorithms_keep_data(self):
        for algorithms in ((2,0,0,0),(4,48,0,0),(22,-1,0,0)):
            result=convert(pack(algorithms=algorithms))
            self.assertEqual(struct.unpack_from('<4i',result,24),algorithms)

    def test_no_material(self):
        result=convert(pack(False));self.assertEqual(struct.unpack_from('<12I',result,40),(0,)*12)

    def test_truncations(self):
        original=pack(algorithms=(4,48,0,0))
        for length in range(len(original)):
            with self.subTest(length=length),self.assertRaises((ValueError,struct.error)):convert(original[:length])

    def test_bad_extent(self):
        original=pack();struct.pack_into('<I',original,len(original)-4,250)
        with self.assertRaises(ValueError):convert(original)

    def test_bad_version_and_tail(self):
        for original in (b'VPM2'+pack()[4:],pack()+b'x'):
            with self.assertRaises(ValueError):convert(original)


if __name__=='__main__':unittest.main()
