#!/usr/bin/env python3
"""Battles for the Bozo runner's debug launcher (--battle), from the game's own event scripts.

VP2 starts a battle with event-script op 0x38 (handler 3a0b90 in mod003): the instruction word (0x38 << 8) | 2, then
arg0 = (stage << 16) | battle id and arg1 = flags | (byte << 24). Scripted battles pass flags 0x01000100; id 0xffff
selects the area's encounter table and flags 0xff carry an inline formation after the instruction, which the launcher
cannot express (both are counted, not listed).

  tools/vp2_battles.py [--assets build_usa/extract/assets] [--out build_usa/bozo/battles.tsv]

Output (game data: build tree only, never committed), one line per distinct (id, stage, flags):
  id  stage  flags  uses  source (first asset file and byte offset)
sorted by id; the runner's --battle takes ID,STAGE,FLAGS.
"""
import argparse
import collections
import os
import struct

OP = struct.pack('<I', 0x3802)


def instructions(data):
    """Every plausible op-0x38 instruction in one asset file: (offset, id, stage, flags)."""
    at = data.find(OP)
    while at >= 0:
        if at % 4 == 0 and at + 12 <= len(data):
            _, arg0, flags = struct.unpack_from('<3I', data, at)
            battle, stage = arg0 & 0xffff, arg0 >> 16
            # Plausible: a stage below 0x8000 and flags with a count byte 1..7 on top and a zero low byte (scripted
            # battles: 0x01000100, 0x01000300, 0x0100ef00; 0xff low bytes are inline formations, counted apart).
            # Other data holding the word 0x3802 (flags 0x702, 0x8002, ...) fails this.
            if stage < 0x8000 and 1 <= (flags >> 24) <= 7 and (flags & 0xff) in (0, 0xff) and (flags & 0xffffff) < 0x10000:
                yield at, battle, stage, flags
        at = data.find(OP, at + 1)


def scan(root):
    found, skipped = collections.OrderedDict(), collections.Counter()
    for folder, _, names in sorted(os.walk(root)):
        for name in sorted(names):
            data = open(os.path.join(folder, name), 'rb').read()
            for at, battle, stage, flags in instructions(data):
                if battle == 0xffff:
                    skipped['area table (id 0xffff)'] += 1
                    continue
                if flags & 0xff == 0xff:
                    skipped['inline formation (flags 0xff)'] += 1
                    continue
                key = (battle, stage, flags)
                if key in found:
                    found[key][0] += 1
                else:
                    found[key] = [1, f'{name}+{at:x}']
    return found, skipped


def main():
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--assets', default=os.path.join(root, 'build_usa', 'extract', 'assets'))
    ap.add_argument('--out', default=os.path.join(root, 'build_usa', 'bozo', 'battles.tsv'))
    a = ap.parse_args()
    found, skipped = scan(a.assets)
    if not found:
        raise SystemExit('no battle instructions found: extract the assets first')
    rows = sorted(found.items(), key=lambda kv: (kv[0][0], -kv[1][0], kv[0]))
    with open(a.out, 'w', encoding='utf-8', newline='\n') as out:
        out.write('# id\tstage\tflags\tuses\tsource\n')
        for (battle, stage, flags), (uses, source) in rows:
            out.write(f'0x{battle:x}\t{stage}\t0x{flags:08x}\t{uses}\t{source}\n')
    print(f'{len({k[0] for k in found})} battle ids, {len(found)} (id, stage, flags) rows -> {a.out}; skipped {dict(skipped)}')


if __name__ == '__main__':
    main()
