import struct,sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from vp2_bozo_particle_materials import convert

def template():
    d=bytearray(0x110);d[0]=d[1]=1
    def put(at,fmt,*values):struct.pack_into('<'+fmt,d,at,*values)
    put(0x1c,'I',0x6c0902e0);put(0xbc,'I',0x6c0502e9)
    for at,reg in ((0x28,5),(0x48,6),(0x58,6),(0x78,0x42),(0x88,0x42),(0x98,0x42),(0xa8,0x42)):put(at,'Q',reg)
    put(0x20,'Q',9);put(0x40,'Q',0x6fedcba98)
    for at,alpha in ((0x70,0x44),(0x80,0x4a),(0x90,0x4a),(0xa0,0x4a)):put(at,'Q',alpha)
    return d

class MaterialTests(unittest.TestCase):
    def test_named_semantics(self):
        packed=convert(template());self.assertEqual(len(packed),116);self.assertEqual(packed[:8],struct.pack('<4sI',b'VPP1',1))
        values=struct.unpack_from('<27I',packed,8)
        self.assertEqual(values[:7],(0,1,0x110,1,0,1,2));self.assertEqual(values[7:12],(0,1,0,1,0));self.assertEqual(values[12:17],(2,2,0,1,0))
    def test_physical_addresses_disappear(self):
        a=template();b=template();struct.pack_into('<I',b,0x40,0x12345678);self.assertEqual(convert(a),convert(b))
    def test_missing_or_truncated_shape(self):
        for d in (b'',template()[:-1],b'xxxx'+template()):self.assertEqual(convert(d),struct.pack('<4sI',b'VPP1',0))
    def test_register_identity_and_count_guards(self):
        for at,value in ((0x28,6),(0x48,5),(0x78,6),(0xbc,0x6c0a02e9),(0x1c,0x6c0802e0)):
            d=template();struct.pack_into('<I',d,at,value);self.assertEqual(convert(d),struct.pack('<4sI',b'VPP1',0))
    def test_offsets_and_independent_records(self):
        d=bytearray(0x20)+template()+bytearray(0x10)+template();p=convert(d)
        self.assertEqual(struct.unpack_from('<I',p,4)[0],2);self.assertEqual(struct.unpack_from('<I',p,8)[0],0x20);self.assertEqual(struct.unpack_from('<I',p,116)[0],0x140)

if __name__=='__main__':unittest.main()
