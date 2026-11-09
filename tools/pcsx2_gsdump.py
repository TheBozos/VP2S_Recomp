"""PCSX2 GS dump reader: header, state, packets; GIF tag decoding into primitives with their GS state."""
import struct, sys
from compression import zstd

def load(path):
    d = open(path, 'rb').read()
    if path.endswith('.zst'): d = zstd.decompress(d)
    return d

def packets(d):
    at = 0
    crc, = struct.unpack_from('<I', d, at); at += 4
    if crc == 0xFFFFFFFF:
        hsize, = struct.unpack_from('<I', d, at); at += 4
        hdr = d[at:at+hsize]; at += hsize
        state_version, state_size, serial_offset, serial_size, crc2, sw, sh, so, ss = struct.unpack_from('<9I', hdr, 0)
        state = d[at:at+state_size]; at += state_size
    else:
        state_size, = struct.unpack_from('<I', d, at); at += 4
        state = d[at:at+state_size]; at += state_size
    regs = d[at:at+8192]; at += 8192
    out = []
    while at < len(d):
        t = d[at]; at += 1
        if t == 0:
            path = d[at]; size, = struct.unpack_from('<I', d, at+1); at += 5
            out.append(('transfer', path, d[at:at+size])); at += size
        elif t == 1:
            out.append(('vsync', d[at])); at += 1
        elif t == 2:
            size, = struct.unpack_from('<I', d, at); at += 4; out.append(('readfifo', size))
        elif t == 3:
            out.append(('regs', d[at:at+8192])); at += 8192
        else:
            raise ValueError(f'bad packet type {t} at {at-1:x}')
    return state, regs, out

REGNAMES = {0:'PRIM',1:'RGBAQ',2:'ST',3:'UV',4:'XYZF2',5:'XYZ2',6:'TEX0_1',7:'TEX0_2',8:'CLAMP_1',9:'CLAMP_2',10:'FOG',
            0xc:'XYZF3',0xd:'XYZ3',0x14:'TEX1_1',0x15:'TEX1_2',0x16:'TEX2_1',0x17:'TEX2_2',0x18:'XYOFFSET_1',0x19:'XYOFFSET_2',
            0x1a:'PRMODECONT',0x1b:'PRMODE',0x1c:'TEXCLUT',0x22:'SCANMSK',0x3b:'TEXA',0x3d:'FOGCOL',0x3f:'TEXFLUSH',
            0x40:'SCISSOR_1',0x41:'SCISSOR_2',0x42:'ALPHA_1',0x43:'ALPHA_2',0x44:'DIMX',0x45:'DTHE',0x46:'COLCLAMP',
            0x47:'TEST_1',0x48:'TEST_2',0x49:'PABE',0x4a:'FBA_1',0x4b:'FBA_2',0x4c:'FRAME_1',0x4d:'FRAME_2',0x4e:'ZBUF_1',0x4f:'ZBUF_2',
            0x50:'BITBLTBUF',0x51:'TRXPOS',0x52:'TRXREG',0x53:'TRXDIR',0x54:'HWREG'}

class Gs:
    """Tracks register state and emits drawing kicks (prim, vertices, state snapshot)."""
    def __init__(self):
        self.r = {}; self.verts = []; self.prims = []; self.q = 1.0; self.frame = 0
    def write(self, reg, val):
        self.r[reg] = val
        if reg == 0:
            self.verts = []
        elif reg == 1:
            self.q = struct.unpack('<f', struct.pack('<I', (val >> 32) & 0xffffffff))[0]
        elif reg in (4, 5, 0xc, 0xd):
            self.vertex(reg, val)
    def vertex(self, reg, val):
        prim = self.r.get(0, 0) if not (self.r.get(0x1a, 1) & 1) == 0 else self.r.get(0x1b, 0)
        prim_type = self.r.get(0, 0) & 7
        x = (val & 0xffff) / 16; y = ((val >> 16) & 0xffff) / 16
        if reg in (4, 0xc): z = (val >> 32) & 0xffffff; f = (val >> 56) & 0xff
        else: z = (val >> 32) & 0xffffffff; f = None
        v = dict(x=x, y=y, z=z, f=f, rgba=self.r.get(1, 0) & 0xffffffff, q=self.q, st=self.r.get(2, 0), uv=self.r.get(3, 0))
        self.verts.append(v)
        need = {0:1, 1:2, 2:2, 3:3, 4:3, 5:3, 6:2}.get(prim_type, 3)
        kick = reg in (4, 5)
        if len(self.verts) >= need:
            if kick:
                ctx = (self.r.get(0, 0) >> 9) & 1
                self.prims.append(dict(type=prim_type, prim=self.r.get(0, 0), verts=list(self.verts[-need:]), frame=self.frame,
                                       alpha=self.r.get(0x42 + ctx, 0), tex0=self.r.get(6 + ctx, 0), test=self.r.get(0x47 + ctx, 0),
                                       fogcol=self.r.get(0x3d, 0), fba=self.r.get(0x4a + ctx, 0), framereg=self.r.get(0x4c + ctx, 0),
                                       zbuf=self.r.get(0x4e + ctx, 0), pabe=self.r.get(0x49, 0), colclamp=self.r.get(0x46, 0),
                                       tex1=self.r.get(0x14 + ctx, 0), xyoff=self.r.get(0x18 + ctx, 0), dthe=self.r.get(0x45, 0)))
            if prim_type in (0, 1, 3, 6):
                self.verts = []
            elif prim_type == 2 or prim_type == 4:
                self.verts = self.verts[-(need - 1):]
            elif prim_type == 5:
                self.verts = [self.verts[0]] + self.verts[-1:]

def gif(gs, data):
    at = 0; n = len(data)
    while at + 16 <= n:
        lo, hi = struct.unpack_from('<QQ', data, at); at += 16
        nloop = lo & 0x7fff; pre = (lo >> 46) & 1; prim = (lo >> 47) & 0x7ff; flg = (lo >> 58) & 3; nreg = (lo >> 60) & 15 or 16
        regs = [(hi >> (4 * i)) & 15 for i in range(nreg)]
        if flg == 0:
            if pre: gs.write(0, prim)
            for _ in range(nloop):
                for reg in regs:
                    if at + 16 > n: return
                    a, b = struct.unpack_from('<QQ', data, at); at += 16
                    if reg == 0xe: gs.write(b & 0xff, a)
                    elif reg == 0: gs.write(0, a & 0x7ff)
                    elif reg == 1: gs.write(1, (a & 0xff) | ((a >> 32 & 0xff) << 8) | ((b & 0xff) << 16) | ((b >> 32 & 0xff) << 24) | (gs.r.get(1, 0) & 0xffffffff00000000))
                    elif reg == 2: gs.write(2, a); gs.q = struct.unpack('<f', struct.pack('<I', b & 0xffffffff))[0]
                    elif reg == 3: gs.write(3, a)
                    elif reg == 4:
                        xyz = (a & 0xffff) | ((a >> 32 & 0xffff) << 16) | (((b >> 4) & 0xffffff) << 32) | (((b >> 36) & 0xff) << 56)
                        gs.write(0xc if (b >> 47) & 1 else 4, xyz)
                    elif reg == 5:
                        xyz = (a & 0xffff) | ((a >> 32 & 0xffff) << 16) | ((b & 0xffffffff) << 32)
                        gs.write(0xd if (b >> 47) & 1 else 5, xyz)
                    elif reg == 0xa: gs.write(0xa, (b >> 36 & 0xff) << 56)
                    elif reg == 0xf: pass
                    else: gs.write(reg, a)
        elif flg == 1:
            if pre: gs.write(0, prim)
            total = nloop * nreg; words = []
            for i in range(total):
                if at + 8 > n: return
                words.append(struct.unpack_from('<Q', data, at)[0]); at += 8
            if total & 1: at += 8
            for i, w in enumerate(words):
                reg = regs[i % nreg]
                if reg == 0xe: continue
                gs.write(reg if reg not in (0xc,) else reg, w)
        else:
            at += nloop * 16

def run(path):
    d = load(path); state, regs, pk = packets(d)
    gs = Gs()
    for p in pk:
        if p[0] == 'transfer': gif(gs, p[2])
        elif p[0] == 'vsync': gs.frame += 1
    return gs, pk

if __name__ == '__main__':
    gs, pk = run(sys.argv[1])
    kinds = {}
    for p in pk: kinds[p[0]] = kinds.get(p[0], 0) + 1
    print(kinds, 'prims', len(gs.prims), 'frames', gs.frame)
