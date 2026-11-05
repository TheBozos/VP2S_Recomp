"""Offline mesh metadata tests: editable copies must not need a runtime VIF decoder."""
import struct
import unittest

import vp2_meshes as meshes


def command(code, count=0, address=0):
    return struct.pack('<I', code << 24 | count << 16 | address)


def packet():
    header = [0] * 28
    header[:4] = [0, 64, 1, 32]
    header[9] = 2
    data = bytearray(command(0x6D, 7) + struct.pack('<28h', *header))
    data += command(0x62, 1, 32) + b'\x02\0\0\0'
    data += command(0x01, address=0x104)  # four rows per vertex
    normal = len(data) + 4
    data += command(0x68, 2, 64) + struct.pack('<6f', 0, 1, 0, 1, 0, 0)
    position = len(data) + 4
    data += command(0x68, 2, 67) + struct.pack('<6f', 1, 2, 3, 4, 5, 6)
    struct.pack_into('<h', data, 4, len(data))
    return data, normal, position


class EditableVertexTests(unittest.TestCase):
    def test_offsets_relative_to_mesh_and_editable_copy(self):
        data, normal, position = packet()
        prefix = b'asset header' * 5
        record, error = meshes.convert(prefix + data, len(prefix), len(data))
        self.assertIsNone(error)
        size, n0, p0, n1, p1 = struct.unpack_from('<5I', record, len(record) - 20)
        self.assertEqual((size, n0, p0, n1, p1), (len(data), normal, position, normal + 12, position + 12))
        work = bytearray(data)
        struct.pack_into('<3f', work, p1, 8, 9, 10)
        self.assertEqual(struct.unpack_from('<3f', work, p1), (8, 9, 10))
        self.assertEqual(struct.unpack_from('<3f', data, p1), (4, 5, 6))

    def test_masked_constant_excludes_xyz_source(self):
        data, _, _ = packet()
        data += command(0x30) + struct.pack('<4I', 1, 2, 3, 4)
        data += command(0x20) + struct.pack('<I', 0x55)
        data += command(0x78, 1, 64) + struct.pack('<3f', 9, 9, 9)
        vu = meshes.Vu()
        self.assertTrue(meshes.unpack(vu, data, 0, len(data)))
        self.assertEqual(meshes.source_row(vu, 64), 0xFFFFFFFF)

    def test_masked_keep_preserves_original_source(self):
        data, normal, _ = packet()
        data += command(0x20) + struct.pack('<I', 0xAA)
        data += command(0x78, 1, 64) + struct.pack('<3f', 9, 9, 9)
        vu = meshes.Vu()
        self.assertTrue(meshes.unpack(vu, data, 0, len(data)))
        self.assertEqual(meshes.source_row(vu, 64), normal)

    def test_scalar_replication_is_not_contiguous_xyz(self):
        data = command(0x60, 1, 64) + struct.pack('<I', 7)
        vu = meshes.Vu()
        self.assertTrue(meshes.unpack(vu, data, 0, len(data)))
        self.assertEqual(meshes.source_row(vu, 64), 0xFFFFFFFF)

    def test_packed_xyz_is_not_a_float_source(self):
        for code, payload in ((0x69, struct.pack('<3h', 1, 2, 3) + b'\0\0'),
                              (0x6A, b'\x01\x02\x03\0'), (0x6F, b'\xFF\x7F\0\0')):
            with self.subTest(code=code):
                data = command(code, 1, 64) + payload
                vu = meshes.Vu()
                self.assertTrue(meshes.unpack(vu, data, 0, len(data)))
                self.assertEqual(meshes.source_row(vu, 64), 0xFFFFFFFF)


if __name__ == '__main__':
    unittest.main()
