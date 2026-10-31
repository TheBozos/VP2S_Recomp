#!/usr/bin/env python3
"""VP2's meshes converted offline to a native format, for the native renderer to load by where the game loaded them.

VP2 keeps its geometry only as prebuilt VIF packets inside its model files ('FPS\\0' chunks: first mesh at model + 0x60,
each mesh one packet whose size is the s16 at +4; docs/vp2_models.md): the unpack codes put a header, the strip
lengths and the vertices (4 qwords each: normal, UV, colour, position) into VU memory, and a material block (rows
7..21) when the mesh carries one. Those packets are read here, once, from the user's own disc (tools/extract.py
--assets); while the game runs nothing reads them: the renderer takes the files this writes (rule decision
2026-10-31, as for the textures). The set-up routines that only change the format (6, 8: fixed point to float) are
applied here; the others (skinning 2 / 4, fade 44, UV scroll 48) depend on the frame and are kept with their data.

  tools/vp2_meshes.py [--assets build/extract/assets] [--out build/vp2_meshes]

Output (game data: build tree only, never committed): <out>/<asset part>.msh (the asset file's name without its
extension), a list of meshes:
  u32 magic 'VPM1', u32 mesh count, then per mesh:
    u32 offset of the mesh in the asset file, u32 vertex count, u32 strip count, u32 flags (header 2.x),
    i32 group mask (header 6.y), i32 routines[4], i32 routine data offsets[4] (rows from the mesh's top),
    f32 bounding centre[3], u32 has material,
    material (16 u32 words when it has one: ALPHA lo/hi, TEX0 lo/hi, strip tag PRIM, cull flag, texture index, then
      the raw rows' x lanes 7..15 for what is not decoded yet),
    u16 strip lengths[strip count] (padded to 4 bytes),
    vertices: f32 position[4] (w: the bone's VU address for routine 2), f32 normal[3], f32 uv[2], u8 rgba[4],
    for routine 4: per vertex i32 bone a, f32 weight a, i32 bone b, f32 weight b.
"""
import argparse
import os
import struct
import sys
from collections import Counter


class Vu:
    """The 1024 quadwords one mesh's unpack codes write (lanes as raw 32-bit words), TOPS = 0."""

    def __init__(self):
        self.mem = [[0, 0, 0, 0] for _ in range(1024)]
        self.cl, self.wl = 1, 1
        self.mask = 0
        self.row = [0, 0, 0, 0]

    def i(self, qw, lane):
        v = self.mem[qw % 1024][lane] & 0xFFFFFFFF
        return v - (1 << 32) if v & 0x80000000 else v

    def f(self, qw, lane):
        return struct.unpack('<f', struct.pack('<I', self.mem[qw % 1024][lane] & 0xFFFFFFFF))[0]

    def setf(self, qw, lane, value):
        self.mem[qw % 1024][lane] = struct.unpack('<I', struct.pack('<f', value))[0]


def unpack(vu, data, at, end):
    """Runs a mesh packet's VIF codes into vu. False on a code this tool does not know."""
    p = at
    while p + 4 <= end:
        w, = struct.unpack_from('<I', data, p)
        cmd, num, imm = (w >> 24) & 0x7F, (w >> 16) & 0xFF, w & 0xFFFF
        p += 4
        if (cmd & 0x60) == 0x60:
            vn, vl = (cmd >> 2) & 3, cmd & 3
            n = num or 256
            unsigned = (imm >> 14) & 1
            masked = (cmd >> 4) & 1
            addr = imm & 0x3FF
            comps = vn + 1
            if vl == 3:  # V4-5
                nbytes = 2 * n
            else:
                nbytes = comps * (32 >> vl) // 8 * n
            for k in range(n):
                qw = (addr + (k // vu.wl) * vu.cl + k % vu.wl) % 1024
                values = []
                if vl == 3:
                    v, = struct.unpack_from('<H', data, p + 2 * k)
                    values = [(v & 0x1F) << 3, ((v >> 5) & 0x1F) << 3, ((v >> 10) & 0x1F) << 3, ((v >> 15) & 1) << 7]
                else:
                    size = 32 >> vl
                    fmt = {32: 'I', 16: 'H', 8: 'B'}[size]
                    for c in range(comps):
                        raw, = struct.unpack_from('<' + fmt, data, p + (k * comps + c) * size // 8)
                        if size < 32 and not unsigned and raw >> (size - 1):
                            raw -= 1 << size
                        values.append(raw & 0xFFFFFFFF)
                    if comps == 1:  # S: one value to every lane
                        values = values * 4
                    elif comps < 4:
                        values += [vu.mem[qw][c] for c in range(comps, 4)]
                for c in range(4):
                    if masked:
                        m = (vu.mask >> (2 * c + 8 * min(k % vu.wl, 3))) & 3
                        if m == 1:
                            vu.mem[qw][c] = vu.row[c]
                            continue
                        if m in (2, 3):
                            continue
                    vu.mem[qw][c] = values[c]
            p += (nbytes + 3) // 4 * 4
            continue
        if cmd == 0x01:  # STCYCL
            vu.cl, vu.wl = imm & 0xFF, (imm >> 8) & 0xFF
            if vu.wl == 0:
                vu.wl = 1
        elif cmd == 0x20:  # STMASK
            vu.mask, = struct.unpack_from('<I', data, p)
            p += 4
        elif cmd == 0x30:  # STROW
            vu.row = list(struct.unpack_from('<4I', data, p))
            p += 16
        elif cmd == 0x31:  # STCOL
            p += 16
        elif cmd in (0x00, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x10, 0x11, 0x13):
            pass
        else:
            return False
    return True


def material(vu):
    """The material block (rows 7..21) the mesh carries: blend, texture template, strip PRIM, cull, texture index."""
    alpha = (vu.mem[8][0], vu.mem[8][1])
    tex0 = (vu.mem[9][0], vu.mem[9][1])
    prim = (vu.mem[13][1] >> 15) & 0x7FF  # the strip tag's PRIM (PRE set)
    cull = vu.mem[15][0]
    texture = vu.mem[15][2]
    rest = [vu.mem[r][0] for r in range(7, 16)]
    return [alpha[0], alpha[1], tex0[0], tex0[1], prim, cull, texture] + rest


def convert(data, at, size):
    vu = Vu()
    if not unpack(vu, data, at, at + size):
        return None, 'unknown VIF code'
    top = 0
    vertices = top + vu.i(top, 1)
    strips = vu.i(top, 2)
    lengths = top + vu.i(top, 3)
    count = vu.i(top + 2, 1)
    flags = vu.i(top + 2, 0)
    group = vu.i(top + 6, 1)
    if not (0 < count < 1024 and 0 < strips < 1024):
        return None, 'bad header'
    routines = [vu.i(top + 5, r) for r in range(4)]
    bases = [vu.i(top + 4, r) for r in range(4)]
    for r in routines:  # the format conversions
        if r == 8:
            for k in range(count):
                v = vertices + 4 * k
                for c in range(3):
                    vu.setf(v, c, vu.i(v, c) / 4096.0)
                    vu.setf(v + 3, c, vu.i(v + 3, c) / 16.0)
        elif r == 6:
            for k in range(count):
                v = vertices + 4 * k
                for c in range(3):
                    vu.setf(v, c, vu.i(v, c) / 4096.0)
        elif r not in (0, 46, 2, 4, 44, 48):
            return None, f'routine {r}'
    out = bytearray()
    has_material = 1 if flags & 1 else 0
    out += struct.pack('<IIIIi4i4i3fI', at, count, strips, flags & 0xFFFFFFFF, group, *routines, *bases,
                       vu.f(top + 1, 0), vu.f(top + 1, 1), vu.f(top + 1, 2), has_material)
    if has_material:
        out += struct.pack('<16I', *[x & 0xFFFFFFFF for x in material(vu)])
    lens = [vu.i(lengths + s, 0) & 0xFFFF for s in range(strips)]
    out += struct.pack(f'<{strips}H', *lens)
    if strips & 1:
        out += b'\0\0'
    for k in range(count):
        v = vertices + 4 * k
        pos = [vu.f(v + 3, c) for c in range(3)] + [float(vu.i(v + 3, 3))]
        normal = [vu.f(v, c) for c in range(3)]
        uv = [vu.i(v + 1, c) / 4096.0 for c in range(2)]
        rgba = [max(0, min(255, vu.i(v + 2, c))) for c in range(4)]
        out += struct.pack('<4f3f2f4B', *pos, *normal, *uv, *rgba)
    if 4 in routines:
        base = top + bases[routines.index(4)]
        for k in range(count):
            e = base + k
            out += struct.pack('<ifif', vu.i(e, 0), vu.f(e, 1), vu.i(e, 2), vu.f(e, 3))
    return bytes(out), None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    ap.add_argument('--assets', default=os.path.join(root, 'build', 'extract', 'assets'))
    ap.add_argument('--out', default=os.path.join(root, 'build', 'vp2_meshes'))
    ap.add_argument('--limit', type=int, default=0, help='stop after this many asset files (a quick check)')
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    results = Counter()
    files = 0
    for folder, _, names in os.walk(a.assets):
        for name in sorted(names):
            path = os.path.join(folder, name)
            data = open(path, 'rb').read()
            found = data.find(b'FPS\x00')
            if found < 0:
                continue
            meshes = []
            while found >= 0:
                model = found + 0x10
                if model + 0x60 > len(data):
                    break
                n, = struct.unpack_from('<I', data, model + 0x34)
                at = model + 0x60
                for _ in range(min(n, 4096)):
                    if at + 8 > len(data):
                        break
                    w0, = struct.unpack_from('<I', data, at)
                    size, = struct.unpack_from('<h', data, at + 4)
                    if (w0 >> 24) & 0x6F != 0x6D or size <= 0:
                        results['not a mesh'] += 1
                        break
                    record, why = convert(data, at, size)
                    if record is None:
                        results[why] += 1
                    else:
                        meshes.append(record)
                        results['converted'] += 1
                    at += size
                found = data.find(b'FPS\x00', found + 4)
            if meshes:
                with open(os.path.join(a.out, os.path.splitext(name)[0] + '.msh'), 'wb') as f:
                    f.write(struct.pack('<4sI', b'VPM1', len(meshes)) + b''.join(meshes))
            files += 1
            if a.limit and files >= a.limit:
                break
        if a.limit and files >= a.limit:
            break
    for k, v in results.most_common():
        print(f'{k}: {v}')


if __name__ == '__main__':
    sys.exit(main())
