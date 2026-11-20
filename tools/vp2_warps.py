#!/usr/bin/env python3
"""Map entry points for the Bozo runner's debug teleport (--warp), from the game's own event scripts.

VP2 changes map with event-script op 0x3a (handler 3af3b0 in mod003): the instruction word (0x3a << 8) | 7, then
map, entry, x, y, z, camera (-1) and facing in degrees. Every area script carries one such instruction per exit, so
scanning the extracted assets (tools/extract.py --assets) for them lists real entry points: positions the game itself
places the party at.

  tools/vp2_warps.py [--assets build_usa/extract/assets] [--out build_usa/bozo/warps.tsv]

Output (game data: build tree only, never committed), one line per distinct (map, entry, x, y, z, facing):
  map  entry  x  y  z  facing  uses  source (first asset file and byte offset)
sorted by map; the runner's --warp MAP takes the most used entry of that map.
"""
import argparse
import collections
import os
import struct

OP = struct.pack('<I', 0x3a07)


def instructions(data):
    """Every plausible op-0x3a instruction in one asset file: (offset, map, entry, x, y, z, facing)."""
    at = data.find(OP)
    while at >= 0:
        if at % 4 == 0 and at + 32 <= len(data):
            _, map_, entry, x, y, z, camera, facing = struct.unpack_from('<8i', data, at)
            if camera == -1 and 0 <= map_ < 0x10000 and 0 <= facing <= 360 and all(abs(v) < 100000 for v in (x, y, z)):
                yield at, map_, entry, x, y, z, facing
        at = data.find(OP, at + 1)


def scan(root):
    found = collections.OrderedDict()
    for folder, _, names in sorted(os.walk(root)):
        for name in sorted(names):
            data = open(os.path.join(folder, name), 'rb').read()
            for at, *key in instructions(data):
                key = tuple(key)
                if key in found:
                    found[key][0] += 1
                else:
                    found[key] = [1, f'{name}+{at:x}']
    return found


def main():
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--assets', default=os.path.join(root, 'build_usa', 'extract', 'assets'))
    ap.add_argument('--out', default=os.path.join(root, 'build_usa', 'bozo', 'warps.tsv'))
    a = ap.parse_args()
    found = scan(a.assets)
    if not found:
        raise SystemExit('no map-change instructions found: extract the assets first')
    rows = sorted(found.items(), key=lambda kv: (kv[0][0], -kv[1][0], kv[0]))
    with open(a.out, 'w', encoding='utf-8', newline='\n') as out:
        out.write('# map\tentry\tx\ty\tz\tfacing\tuses\tsource\n')
        for (map_, entry, x, y, z, facing), (uses, source) in rows:
            out.write(f'{map_}\t{entry}\t{x}\t{y}\t{z}\t{facing}\t{uses}\t{source}\n')
    print(f'{len({k[0] for k in found})} maps, {len(found)} entry points -> {a.out}')


if __name__ == '__main__':
    main()
