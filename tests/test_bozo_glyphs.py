"""Offline glyph conversion boundary tests; synthetic assets only."""
from pathlib import Path
import struct
import sys
import unittest
import hashlib
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from vp2_glyphs import inspect_font,convert,HEADER,MAGIC

def source():
    data=bytearray(0x90);data[:8]=b'mcps2lib';struct.pack_into('<I',data,0x20,len(data))
    struct.pack_into('<8I',data,0x30,0x80,2,3,3,2,3,2,1)
    data[0x80:0x86]=bytes([0x21,0x43,0x65,0x87,0xa9,0xcb]);return data

class Glyphs(unittest.TestCase):
    def test_nibbles_palette_parts_padding_and_native_alpha(self):
        data=source();palette=[0x20000000+i for i in range(256)]
        result=convert(data,palette)
        self.assertEqual(HEADER.unpack_from(result),(MAGIC,3,2,2,3,4,2,1,2,2))
        pixels=struct.unpack_from('<32I',result,HEADER.size)
        self.assertEqual(pixels[:8],(palette[17],palette[18],palette[19],0,palette[20],palette[21],palette[22],0))
        self.assertEqual(pixels[8:16],(palette[23],palette[24],palette[25],0,palette[26],palette[27],palette[28],0))
        self.assertEqual(pixels[16],palette[33])

    def test_fontless_bundle(self):
        data=source();struct.pack_into('<I',data,0x34,0);self.assertIsNone(inspect_font(data))
        with self.assertRaises(ValueError):convert(data,list(range(256)))

    def test_invalid_bounds_and_metrics_reject(self):
        for at,bad in ((0x20,0xffffff),(0x30,0xffffff),(0x34,16385),(0x3c,2),(0x40,3),(0x44,257),(0x48,0),(0x4c,17)):
            data=source();struct.pack_into('<I',data,at,bad)
            with self.subTest(at=at),self.assertRaises(ValueError):inspect_font(data)
        for data in (b'',bytes(128)):
            with self.assertRaises(ValueError):inspect_font(data)
        with self.assertRaises(ValueError):convert(source(),list(range(255)))

if __name__=='__main__':
    if len(sys.argv)==3 and sys.argv[1]=='--emit':
        root=Path(sys.argv[2]);root.mkdir(parents=True,exist_ok=True)
        packed=convert(source(),[0x20000000+i for i in range(256)])
        name='g_'+hashlib.sha1(packed).hexdigest()+'.vpg'
        (root/name).write_bytes(packed)
        text=f'VPGFM1\n21551000 0 80 {name}\n';(root/'manifest.txt').write_text(text)
        for directory,data in (('bad_header',b'WRONG000'+packed[8:]),('bad_size',packed[:-4]),('extra',packed+b'x')):
            child=root/directory;child.mkdir(exist_ok=True);(child/name).write_bytes(data);(child/'manifest.txt').write_text(text)
        child=root/'bad_manifest';child.mkdir(exist_ok=True);(child/'manifest.txt').write_text('VPGFM1\n21551000 0 80 ../wrong.vpg\n')
        geometry=root/'geometry';geometry.mkdir(exist_ok=True)
        data=bytearray(0xb0);data[:8]=b'mcps2lib';struct.pack_into('<I',data,0x20,len(data))
        struct.pack_into('<8I',data,0x30,0x80,2,6,18,4,6,6,1)
        data[0x80:0xa4]=bytes([0x21]*36)
        packed=convert(data,[0x20000000+i for i in range(256)])
        name='g_'+hashlib.sha1(packed).hexdigest()+'.vpg';(geometry/name).write_bytes(packed)
        (geometry/'manifest.txt').write_text(f'VPGFM1\n21551000 0 80 {name}\n')
    else:unittest.main()
