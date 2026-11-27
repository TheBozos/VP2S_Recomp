#!/usr/bin/env python3
"""List VP2's field encounters (the visible enemies' battles) from the extracted assets.

A visible enemy (mod003 388900) finds its encounter record with 3eda30: from the enemy's script offset (+1c) it walks
the event script (op word = op << 8 | argument count) to the first op 0x38. An encounter is op 0x38 with an inline
formation table (arg1's low byte 0xff; tools/vp2_battles.py lists the plain battles):

  +0   0x38NN                 NN arguments: 2 + 4 * groups (1 group below 7, else 4)
  +4   stage << 16 | battle   the defaults (stage 0: the field's own, its +ae)
  +8   count << 24 | 0xff     the default enemy count
  +12  groups of 16 bytes:
         4 halfwords  battle ids (0xffff: empty)
         4 bytes      enemy counts
         4 bytes      weights: low 7 bits the weight (0: empty); bit 7 puts the entry in the second pool

The enemy draws one battle from each pool (393630, by weight; the defaults when a pool is empty) into +984 / +988 with
their counts in +994 / +995, and op 0x38's handler (3a0b90) draws again when the enemy (flag 0x20) starts the battle.
The table has one row per battle id: its stage (0 = the field's), its flags as the handler passes them (arg1 without
its 0xff byte: the largest enemy count on top, no 0x0100 bit, which the scripted battles carry), how
many entries name it, and the first source.

  python -I tools/vp2_encounters.py [--assets build_usa/extract/assets] [--out build_usa/bozo/encounters.tsv]
"""
import argparse
import collections
import os
import struct


def records(data):
    """Every plausible encounter record in one asset file: (offset, stage, [(battle, count, weight, second pool)])."""
    for at in range(0, len(data) - 28, 4):
        word = struct.unpack_from('<I', data, at)[0]
        if word >> 8 != 0x38:
            continue
        count = word & 0xff
        groups = 1 if count < 7 else 4
        if count != 2 + 4 * groups or at + 12 + 16 * groups > len(data):
            continue
        arg0, arg1 = struct.unpack_from('<2I', data, at + 4)
        stage = arg0 >> 16
        if arg1 & 0xff != 0xff or stage >= 0x8000 or not 1 <= arg1 >> 24 <= 7:
            continue
        entries, ok = [], True
        for g in range(groups):
            base = at + 12 + 16 * g
            values = struct.unpack_from('<4H', data, base)
            for v, c, w in zip(values, data[base + 8:base + 12], data[base + 12:base + 16]):
                if v == 0xffff or not w & 0x7f:
                    continue
                if not 1 <= c <= 7 or v >= 0x400:
                    ok = False
                entries.append((v, c, w & 0x7f, bool(w & 0x80)))
        if ok and entries:
            yield at, stage, entries


def main():
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--assets', default=os.path.join(root, 'build_usa', 'extract', 'assets'))
    ap.add_argument('--out', default=os.path.join(root, 'build_usa', 'bozo', 'encounters.tsv'))
    a = ap.parse_args()
    rows = collections.OrderedDict()
    tables = 0
    for folder, _, names in sorted(os.walk(a.assets)):
        for name in sorted(names):
            data = open(os.path.join(folder, name), 'rb').read()
            for at, stage, entries in records(data):
                tables += 1
                for battle, count, _, _ in entries:
                    row = rows.setdefault(battle, {'stage': stage, 'count': 0, 'uses': 0, 'source': f'{name}+{at:x}'})
                    row['count'] = max(row['count'], count)
                    row['uses'] += 1
    with open(a.out, 'w', encoding='utf-8', newline='\n') as out:
        out.write('# battle\tstage\tflags\tuses\tsource\n')
        for battle, row in rows.items():
            out.write(f'0x{battle:x}\t{row["stage"]}\t0x{row["count"]:02x}000000\t{row["uses"]}\t{row["source"]}\n')
    print(f'{tables} encounter tables; {len(rows)} battle ids -> {a.out}')


if __name__ == '__main__':
    main()
