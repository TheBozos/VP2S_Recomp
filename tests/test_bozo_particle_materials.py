import struct,sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from vp2_bozo_particle_materials import convert

def template():
    d=bytearray(0x110);d[0]=d[1]=1
    def put(at,fmt,*values):struct.pack_into('<'+fmt,d,at,*values)
    put(0x1c,'I',0x6c0902e0);put(0xbc,'I',0x6c0502e9)
    for at,reg in ((0x28,5),(0x48,6),(0x58,6),(0x78,0x42),(0x88,0x42),(0x98,0x42),(0xa8,0x42)):put(at,'Q',reg)
    put(0x20,'Q',0x300000002);put(0x40,'Q',0x6fedcba98)
    for at,alpha in ((0x70,0x44),(0x80,0x4a),(0x90,0x4a),(0xa0,0x4a)):put(at,'Q',alpha)
    return d

class MaterialTests(unittest.TestCase):
    def test_named_semantics(self):
        packed=convert(template());self.assertEqual(len(packed),120);self.assertEqual(packed[:8],struct.pack('<4sI',b'VPP3',1))
        values=struct.unpack_from('<28I',packed,8)
        self.assertEqual(values[:8],(0,1,0x110,1,0,2,3,1));self.assertEqual(values[8:13],(0,1,0,1,0));self.assertEqual(values[13:18],(2,2,0,1,0))
    def test_pass_count_bounds(self):
        for passes in (0,5):
            d=template();struct.pack_into('<I',d,0x20,passes);self.assertEqual(convert(d),struct.pack('<4sI',b'VPP3',0))
        d=template();struct.pack_into('<I',d,0x20,4);self.assertEqual(struct.unpack_from('<28I',convert(d),8)[5],4)
    def test_kind_stride_per_pass(self):
        # z = 25 qwords: five kinds per pass (a 5x2 grid, two passes).
        d=bytearray(0xc0+80*10);d[:0xc0]=template()[:0xc0];d[0]=5;d[1]=2
        struct.pack_into('<I',d,0xbc,0x6c3202e9);struct.pack_into('<I',d,0x28,25)
        self.assertEqual(struct.unpack_from('<28I',convert(d),8)[5:8],(2,3,5))
        for z,passes in ((24,2),(0,1)):   # not whole kinds, none
            e=bytearray(d);struct.pack_into('<II',e,0x20,passes,3);struct.pack_into('<I',e,0x28,z);self.assertEqual(convert(e),struct.pack('<4sI',b'VPP3',0))
        e=bytearray(d);struct.pack_into('<I',e,0x2c,1);self.assertEqual(convert(e),struct.pack('<4sI',b'VPP3',0))
    def test_physical_addresses_disappear(self):
        a=template();b=template();struct.pack_into('<I',b,0x40,0x12345678);self.assertEqual(convert(a),convert(b))
    def test_missing_or_truncated_shape(self):
        for d in (b'',template()[:-1],b'xxxx'+template()):self.assertEqual(convert(d),struct.pack('<4sI',b'VPP3',0))
    def test_register_identity_and_count_guards(self):
        for at,value in ((0x28,6),(0x28,0),(0x48,5),(0x78,6),(0xbc,0x6c0a02e9),(0x1c,0x6c0802e0)):
            d=template();struct.pack_into('<I',d,at,value);self.assertEqual(convert(d),struct.pack('<4sI',b'VPP3',0))
    def test_offsets_and_independent_records(self):
        d=bytearray(0x20)+template()+bytearray(0x10)+template();p=convert(d)
        self.assertEqual(struct.unpack_from('<I',p,4)[0],2);self.assertEqual(struct.unpack_from('<I',p,8)[0],0x20);self.assertEqual(struct.unpack_from('<I',p,120)[0],0x140)

if __name__=='__main__':unittest.main()
