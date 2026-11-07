"""Game-free tests for ELF metadata preservation and carved entry handling."""
import struct
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
from vp2_bozo_prepare import EH, PH, SH, SY, elf_parts, load_identity, prepare, sdk_names,static_call_targets,linked_call_entries,literal_vtable_targets

BASE = 0x100000


def fixture():
    names = b'\0.text\0.shstrtab\0'
    image = struct.pack('<6I', 0x24020007, 0x03e00008, 0, 0x24420001, 0x03e00008, 0) + bytes(104)
    names_at = 0x100 + len(image)
    sh_at = (names_at + len(names) + 3) & ~3
    header = EH.pack(b'\x7fELF\x01\x01\x01'+bytes(9), 2, 8, 1, BASE, EH.size, sh_at,
                     0x20924000, EH.size, PH.size, 1, SH.size, 3, 2)
    segment = PH.pack(1, 0x100, BASE, BASE, len(image), len(image)+16, 7, 16)
    result = header + segment + bytes(0x100-len(header)-len(segment)) + image + names
    result += bytes(sh_at-len(result))
    result += SH.pack(*([0]*10))
    result += SH.pack(1, 1, 6, BASE, 0x100, len(image), 0, 0, 4, 0)
    result += SH.pack(7, 3, 0, 0, names_at, len(names), 0, 0, 1, 0)
    rows = [dict(Name='primary', Start=hex(BASE), End=hex(BASE+24)),
            dict(Name='interior', Start=hex(BASE+12), End=hex(BASE+24))]
    return result, rows


class PrepareTests(unittest.TestCase):
    def test_literal_vtable_stores_and_clobbers(self):
        original,_=fixture();data=bytearray(original)
        def word(at,value):struct.pack_into('<I',data,0x100+at,value)
        # A constructor stores the vtable in a nonzero base-class subobject.
        word(0,0x3c020010);word(4,0x24420040);word(8,0xac820010)
        word(12,0x03e00008);word(16,0)
        for at in (32,48):word(at,0x03e00008);word(at+4,0)
        word(64,0);word(68,0);word(72,BASE+32);word(76,BASE+48);word(80,0)
        rows=[dict(Name='constructor',Start=hex(BASE),End=hex(BASE+20))]
        expected={BASE+32,BASE+48}
        self.assertEqual(literal_vtable_targets([(data,rows)],data),expected)
        # No raw scan of data/code; only admitted constructor instructions.
        self.assertEqual(literal_vtable_targets([(data,[])],data),set())
        for at,value in ((8,0),(4,0x8c820040),(4,0x0c040008),(64,1),(72,BASE+33),(76,BASE+0x10000)):
            changed=bytearray(data);struct.pack_into('<I',changed,0x100+at,value)
            with self.subTest(at=at,value=value):
                self.assertEqual(literal_vtable_targets([(changed,rows)],changed),set())
        additions=linked_call_entries(data,rows,expected)
        output,_=prepare(data,rows+additions,{})
        self.assertEqual(load_identity(output),load_identity(data))

    def test_vtable_nomination_is_scoped_to_constructor_image(self):
        original,_=fixture();table=bytearray(original)
        for at,target in ((64,32),(96,40)):
            struct.pack_into('<5I',table,0x100+at,0,0,BASE+target,BASE+target+16,0)
        a=bytearray(table);b=bytearray(table)
        for image,offset in ((a,64),(b,96)):
            struct.pack_into('<5I',image,0x100,0x3c020010,0x24420000|offset,0xac820000,0x03e00008,0)
        rows=[dict(Name='constructor',Start=hex(BASE),End=hex(BASE+20))]
        # Overlapping address ranges do not give the other image's constructor
        # permission to nominate methods for this image.
        self.assertEqual(literal_vtable_targets([(a,rows),(b,[])],table),{BASE+32,BASE+48})
        self.assertEqual(literal_vtable_targets([(b,rows),(a,[])],table),{BASE+40,BASE+56})

    def test_primary_labels_and_load_bytes(self):
        original, rows = fixture()
        output, info = prepare(original, rows, {})
        self.assertEqual(load_identity(original), load_identity(output))
        self.assertEqual((info['functions'], info['entries']), (1, 2))
        self.assertEqual(info['code'], [[BASE, BASE+24]])
        _, _, sections = elf_parts(output)
        table = next(s for s in sections if s[1] == 2)
        symbols = [SY.unpack_from(output, table[4]+i) for i in range(SY.size, table[5], SY.size)]
        self.assertEqual([(s[1], s[2], s[3]) for s in symbols], [(BASE, 24, 0x12), (BASE+12, 0, 0x10)])
        self.assertEqual(prepare(original, rows, {})[0], output)

    def test_zero_size_sdk_alias_does_not_split_function(self):
        original, rows = fixture()
        output, info = prepare(original, rows, {BASE+12: 'native_fixture'})
        self.assertEqual(info['functions'], 1)
        _, _, sections = elf_parts(output)
        table = next(s for s in sections if s[1] == 2)
        symbols = [SY.unpack_from(output, table[4]+i) for i in range(SY.size, table[5], SY.size)]
        self.assertTrue(any(s[1] == BASE+12 and s[2] == 0 and s[3] == 0x12 for s in symbols))

    def test_invalid_inputs_fail(self):
        original, rows = fixture()
        for data in [b'', b'bad ELF', bytes(80)]:
            with self.assertRaises(ValueError): prepare(data, rows, {})
        with self.assertRaises(ValueError): prepare(original, rows, {BASE+0x100: 'outside'})
        with self.assertRaises(ValueError): prepare(original, [dict(Name='bad', Start=hex(BASE+1), End=hex(BASE+24))], {})

    def test_sdk_candidates_are_not_automatically_bound(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp)/'sdk.toml'
            path.write_text('stubs = ["native@0x100000"]\nuntracked_stubs = ["guess@0x100004"]\n')
            self.assertEqual(sdk_names(path), {BASE: 'native'})

    def test_cross_image_static_calls_and_bounded_helpers(self):
        original,rows=fixture();data=bytearray(original)
        def word(offset,value):struct.pack_into('<I',data,0x100+offset,value)
        word(0,0x0c000000|((BASE+32)>>2))
        word(32,0x10800003);word(36,0) # Forward path after the first return.
        word(40,0x03e00008);word(44,0x24020001)
        word(48,0x0c000000|((BASE+64)>>2));word(52,0)
        word(56,0x03e00008);word(60,0)
        word(64,0x08000000|((BASE+80)>>2));word(68,0)
        word(80,0x03e00008);word(84,0x24020002)
        word(96,0x0c000000|((BASE+112)>>2)) # Unadmitted bytes never seed code.
        self.assertEqual(static_call_targets(data,rows),{BASE+32})
        entries=linked_call_entries(data,rows,{BASE+4,BASE+32,BASE+96,BASE+512},{BASE+96})
        spans={(int(r['Start'],0),int(r['End'],0)) for r in entries}
        self.assertEqual(spans,{(BASE+4,BASE+8),(BASE+32,BASE+64),(BASE+64,BASE+72),(BASE+80,BASE+88)})
        output,info=prepare(data,rows+entries,{})
        self.assertEqual(load_identity(data),load_identity(output))
        self.assertEqual(info['code'],[[BASE,BASE+24],[BASE+32,BASE+72],[BASE+80,BASE+88]])
        self.assertEqual(linked_call_entries(data,rows+entries,{BASE+32}),[])


if __name__ == '__main__':
    if len(sys.argv) == 3 and sys.argv[1] == '--emit':
        data, rows = fixture()
        Path(sys.argv[2]).write_bytes(prepare(data, rows, {})[0])
        Path(sys.argv[2]+'.entries').write_text(f'0x{BASE+12:x}\n')
    else:
        unittest.main()
