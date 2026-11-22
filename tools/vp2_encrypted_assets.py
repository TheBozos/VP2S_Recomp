#!/usr/bin/env python3
"""Extract VP2's stream-encrypted disc files (the battle core's loader keys) as ordinary asset parts.

Some battle resources are stored encrypted on the disc. The battle loader (mod028 3936f0) reads a file, looks its
first word up in a six-entry table at 0x50fca0 (pairs of state, tag) and calls the battle core's decrypt job
(mod027 3698f0) with that state, multiplier 0xceb3, increment 0x1c7b, shift 5 and mode 2 (16-bit lanes): for each
halfword k, s = 0xceb3 * s + 0x1c7b (mod 2^32) and the halfword is XORed with (s >> 5) & 0xffff. The decrypted
file is an ordinary SLZ/SLE chain, which the game then decodes part by part. tools/extract.py finds chains by their
plain headers, so it never sees these.

This tool scans every sector start for the six tags, decrypts, requires a plausible SLZ/SLE chain, and writes each
decoded part as <assets>/<offset >> 28 in hex>/<offset:010x>_<part:03d>.<ext>, the extractor's naming (the offset is
the file's disc byte offset, the origin the port records for it). Game data: build tree only, never committed.

  python -I tools/vp2_encrypted_assets.py <game.iso> <mod028.elf> <assets dir>

Other decrypt-job callers (mod028 36e32c, 3b7058, 3ce700: mode 4, keys from other sources) are not covered.
"""
import argparse
import hashlib
import os
import struct
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import slz  # noqa: E402
from extract import EXT, classify  # noqa: E402

SECTOR = 2048
TABLE, ENTRIES = 0x50fca0, 6
MULTIPLIER, INCREMENT, SHIFT = 0xceb3, 0x1c7b, 5
BLOCK = 1 << 16       # halfwords per keystream block


def key_table(elf: bytes):
    ph = struct.unpack_from('<I', elf, 0x1c)[0]
    for i in range(struct.unpack_from('<H', elf, 0x2c)[0]):
        t, o, v, _, fs, _ = struct.unpack_from('<6I', elf, ph + 32 * i)
        if t == 1 and v <= TABLE and TABLE + 8 * ENTRIES <= v + fs:
            raw = elf[o + TABLE - v:o + TABLE - v + 8 * ENTRIES]
            words = struct.unpack(f'<{2 * ENTRIES}I', raw)
            return {words[2 * k + 1]: words[2 * k] for k in range(ENTRIES)}, hashlib.sha256(raw).hexdigest()
    raise SystemExit('mod028 image has no key table at 0x50fca0')


# Keystream by blocks: for k = 1..BLOCK, s_k = A^k s_0 + C (A^(k-1) + ... + 1) (mod 2^32).
_powers = np.empty(BLOCK, np.uint64)
_sums = np.empty(BLOCK, np.uint64)
_p, _s = 1, 0
for _k in range(BLOCK):
    _p = (_p * MULTIPLIER) & 0xffffffff
    _s = (_s * MULTIPLIER + INCREMENT) & 0xffffffff
    _powers[_k], _sums[_k] = _p, _s


def decrypt(data: bytes, state: int) -> bytes:
    """Mode 2, shift 5, from the start of the file (an odd final byte is left as is)."""
    halves = np.frombuffer(data[:len(data) & ~1], np.uint16).copy()
    out = np.empty_like(halves)
    for at in range(0, len(halves), BLOCK):
        n = min(BLOCK, len(halves) - at)
        s = (_powers[:n] * np.uint64(state) + _sums[:n]) & np.uint64(0xffffffff)
        out[at:at + n] = halves[at:at + n] ^ ((s >> np.uint64(SHIFT)) & np.uint64(0xffff)).astype(np.uint16)
        state = int(s[n - 1])
    return out.tobytes() + data[len(data) & ~1:]


def chain(read, state: int):
    """The decrypted file's chain: (buffer, member offsets) or None when it is not one."""
    size = 1 << 16
    while True:
        buf = decrypt(read(size), state)
        members = [0]
        while True:
            at = members[-1]
            if at + 16 > len(buf):
                break
            magic, typ, comp, decomp, nxt = slz.HEADER.unpack_from(buf, at)
            if magic not in (b'SLZ', b'SLE') or typ > 3 or not (0 < comp <= 16 << 20 and 0 < decomp <= 64 << 20) or comp > decomp + 64:
                return None
            if at + 16 + comp > len(buf):
                break
            if not nxt:
                return buf, members
            if not (16 + comp <= nxt <= 16 + comp + 4096):
                return None
            members.append(at + nxt)
        if size >= 256 << 20:
            return None
        size *= 4


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('iso')
    ap.add_argument('module', help='the USA mod028 ELF (its key table)')
    ap.add_argument('assets')
    a = ap.parse_args()
    keys, digest = key_table(open(a.module, 'rb').read())
    print(f'key table {digest[:16]}: {len(keys)} tags')
    size = os.path.getsize(a.iso)
    found = written = 0
    with open(a.iso, 'rb') as f:
        step = 64 << 20
        for base in range(0, size, step):
            f.seek(base)
            block = f.read(min(step, size - base))
            heads = np.frombuffer(block[:len(block) - len(block) % SECTOR], np.uint32)[::SECTOR // 4]
            for k in np.nonzero(np.isin(heads, np.array(list(keys), np.uint32)))[0]:
                at = base + int(k) * SECTOR
                tag = int(heads[k])

                def read(n, at=at):
                    f.seek(at)
                    return f.read(min(n, size - at))
                result = chain(read, keys[tag])
                if not result:
                    continue
                buf, members = result
                found += 1
                for n, m in enumerate(members):
                    data = slz.decompress_chunk(buf, m, slz.SLE_KEY)
                    kind, _ = classify(data, set())
                    # 'p@Ck' packs are named as tools/vp2_archives.py names them.
                    ext = 'pack' if data[:4] == b'p@Ck' else EXT.get(kind, kind[4:].strip().lower() or 'bin')
                    sub = os.path.join(a.assets, f'{at >> 28:x}')
                    os.makedirs(sub, exist_ok=True)
                    path = os.path.join(sub, f'{at:010x}_{n:03d}.{ext}')
                    if os.path.exists(path):
                        if open(path, 'rb').read() != data:
                            raise SystemExit(f'{path} exists with other content')
                        continue
                    with open(path, 'wb') as o:
                        o.write(data)
                    written += 1
                print(f'{at:010x}: tag {tag:08x} state {keys[tag]:x}, {len(members)} parts')
    print(f'{found} encrypted files, {written} parts written')


if __name__ == '__main__':
    main()
