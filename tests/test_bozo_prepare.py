"""Game-free tests for ELF metadata preservation and carved entry handling."""
import struct
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
from vp2_bozo_prepare import EH, PH, SH, SY, elf_parts, load_identity, prepare, sdk_names

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


if __name__ == '__main__':
    if len(sys.argv) == 3 and sys.argv[1] == '--emit':
        data, rows = fixture()
        Path(sys.argv[2]).write_bytes(prepare(data, rows, {})[0])
        Path(sys.argv[2]+'.entries').write_text(f'0x{BASE+12:x}\n')
    else:
        unittest.main()
