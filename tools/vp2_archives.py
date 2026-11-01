#!/usr/bin/env python3
"""VP2's encrypted archives ('p@Ck', the battle data among them) unpacked offline, beside tools/extract.py's assets.

329 files of the game's archive are packs whose member table and members are encrypted with a per-file key k
(mod027 FUN_00366A00 / FUN_00365F70): extract.py's scan for SLZ chains cannot see inside them, so the battle's models and
textures had no asset. This finds each pack's k from its own table, decrypts the members the game decrypts, and writes
every SLZ part found in them as build/extract/assets/<nibble>/<chain disc offset:010x>_<part:03d>.<ext>, the name the
port computes at run time from the game's own reads (port/vp2s_loads.cpp). Game data: build tree only, never committed.

  tools/vp2_archives.py <game.iso> [--out build/extract/assets]

Formats (docs/vp2_models.md):
  TOC: 0x9000 bytes at disc 0x200000 (sector 0x400), three tables of 0xC00 u32 (start sector, sectors, other), XOR'd
       by a stream seeded 0x49287491 (FUN_0011EBC0).
  Pack: entry table at +8 of {u32 offset | 0x80000000 encrypted, u32 id}, ending {end, 0}, each word XOR'd by
       a = a * 2053 + 0x3619 from a = (k ^ 0x5E) * 29. Encrypted members, in order, through one word stream:
       A = (k ^ 0xF3) * 0x2AB7D + 0x33E, V = (k ^ 0xC7) * 3, P = A * 129 + 0x1CBC; per word sh = (V >> 16) & 31,
       out = (w ^ P ^ (A << sh)) - (V >> 1), A += 0x718331, V = V * 1056777 + 0x40001, P = out.
"""
import argparse
import os
import re
import struct
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import slz  # noqa: E402

M = 0xFFFFFFFF
SECTOR = 2048


def s32(x):
    x &= M
    return x - (1 << 32) if x & 0x80000000 else x


def read_toc(iso):
    iso.seek(0x200000)
    raw = iso.read(0x9000)
    t = list(struct.unpack('<%dI' % (0x9000 // 4), raw))
    seed = 0x49287491
    x = seed
    for i in range(0xC00):
        t[i] ^= x
        x = (x ^ (x << 1)) & M
        t[0xC00 + i] ^= x
        y = (~x ^ seed) & M
        t[0x1800 + i] ^= y
        x = (y ^ (y << 2) ^ seed) & M
    t[0] = 0x400
    return [(t[i] & 0xFFFFFF, t[0xC00 + i] & 0xFFFFFF) for i in range(0xC00)]


def entry_table(d, k, size=None):
    a0 = ((k ^ 0x5E) * 29) & M
    x = (a0 * 2053 + 0x3619) & M
    if len(d) < 16:
        return None
    first = (struct.unpack_from('<I', d, 8)[0] ^ x) & 0x7FFFFFFF
    n = first // 4 - 2
    if not (2 <= n < 4096) or n & 1 or 8 + 4 * n > len(d):
        return None
    w = list(struct.unpack_from('<%dI' % n, d, 8))
    a = a0
    for i in range(n):
        a = (a * 2053 + 0x3619) & M
        w[i] ^= a
    entries = [(w[2 * i], w[2 * i + 1]) for i in range(n // 2)]
    while len(entries) > 2 and entries[-1] == (0, 0):  # the table padded to its size after the terminator
        entries.pop()
    offsets = [e[0] & 0x7FFFFFFF for e in entries]
    if offsets[0] != 8 + 4 * n or offsets[0] < 0x18 or any(b < a for a, b in zip(offsets, offsets[1:])):
        return None
    if entries[-1][1] != 0 or abs(offsets[-1] - (size or len(d))) > 2 * SECTOR:
        return None
    return entries


class Stream:
    def __init__(self, k):
        self.A = ((k ^ 0xF3) * 0x2AB7D + 0x33E) & M
        self.V = ((k ^ 0xC7) * 3) & M
        self.P = (self.A * 129 + 0x1CBC) & M

    def decrypt(self, data):
        n = len(data) // 4
        w = struct.unpack('<%dI' % n, data[:4 * n])
        out = []
        A, V, P = self.A, self.V, self.P
        for x in w:
            sh = (s32(V) >> 16) & 0x1F
            t4 = (A << sh) & M
            A = (A + 0x718331) & M
            o = ((x ^ P ^ t4) - (s32(V) >> 1)) & M
            V = (V * 1056777 + 0x40001) & M
            out.append(o)
            P = o
        self.A, self.V, self.P = A, V, P
        return struct.pack('<%dI' % n, *out) + data[4 * n:]


CHAIN = re.compile(rb'S(LZ|LE)[\x00-\x03]')


def chains(data):
    """Every SLZ chain in data: (offset, [decompressed parts])."""
    pos = 0
    while True:
        m = CHAIN.search(data, pos)
        if not m:
            return
        p = m.start()
        _, typ, comp, dsz, nxt = struct.unpack_from('<3sB3I', data, p)
        if not (0 < comp < 16 << 20 and 0 < dsz < 64 << 20):
            pos = p + 1
            continue
        parts, q = [], p
        try:
            while True:
                parts.append(slz.decompress_chunk(data, q, slz.SLE_KEY))
                n = struct.unpack_from('<I', data, q + 12)[0]
                if not n:
                    break
                q += n
        except Exception:
            pos = p + 1
            continue
        yield p, parts
        pos = q + 16


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    ap.add_argument('iso')
    ap.add_argument('--out', default=os.path.join(root, 'build', 'extract', 'assets'))
    a = ap.parse_args()
    if slz.SLE_KEY is None:
        print('no SLE key (VP2S_SLE_KEY or build/sle_key.txt)')
        return 1
    iso = open(a.iso, 'rb')
    toc = read_toc(iso)
    packs = written = 0
    for fid, (lba, sectors) in enumerate(toc):
        if fid == 0 or sectors == 0 or sectors > 0x8000:
            continue
        iso.seek(lba * SECTOR)
        head = iso.read(min(sectors * SECTOR, 0x10000))
        key = next((k for k in range(256) if entry_table(head, k, sectors * SECTOR)), None)
        if key is None:
            continue
        iso.seek(lba * SECTOR)
        d = iso.read(sectors * SECTOR)
        entries = entry_table(d, key)
        if not entries:
            continue
        packs += 1
        stream = Stream(key)
        for i, (offset, ident) in enumerate(entries[:-1]):
            start, end = offset & 0x7FFFFFFF, entries[i + 1][0] & 0x7FFFFFFF
            member = d[start:end]
            pieces = []  # (offset in the file, bytes) to scan for chains
            if offset & 0x80000000:
                pieces.append((start, stream.decrypt(member)))
            elif (ident & 0xFF00) == 0x5000:
                # A plain member of type 0x5000 is a nested pack (mod028 FUN_003B6E30's slots 3..6): its own key, a
                # fresh stream, its encrypted members holding the chains (the battle characters).
                inner = next((k for k in range(256) if entry_table(member, k, len(member))), None)
                sub = entry_table(member, inner, len(member)) if inner is not None else None
                if sub:
                    s2 = Stream(inner)
                    for j, (soffset, _) in enumerate(sub[:-1]):
                        sstart, send = soffset & 0x7FFFFFFF, sub[j + 1][0] & 0x7FFFFFFF
                        piece = member[sstart:send]
                        if soffset & 0x80000000:
                            piece = s2.decrypt(piece)
                        pieces.append((start + sstart, piece))
            else:
                pieces.append((start, member))
            for base, piece in pieces:
              for at, parts in chains(piece):
                disc = lba * SECTOR + base + at
                folder = os.path.join(a.out, '%x' % (disc >> 28))
                os.makedirs(folder, exist_ok=True)
                for n, part in enumerate(parts):
                    ext = 'pack' if part[:4] == b'p@Ck' else 'bin'
                    path = os.path.join(folder, '%010x_%03d.%s' % (disc, n, ext))
                    if not os.path.exists(path):
                        with open(path, 'wb') as f:
                            f.write(part)
                        written += 1
    print(f'packs: {packs}, parts written: {written}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
