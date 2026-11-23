#!/usr/bin/env python3
"""Item names for the F2 cheats' item search, from your own disc's assets (game data: build tree only).

The battle and town message files (mcps) hold every item name, in item-id order (id 0 "Long Sword" .. 750),
NUL-separated, in the game's text encoding: byte + 0x1f for letters and digits, with 0x0b '-', 0x0e ' ',
0x0f and 0x14 an apostrophe, 0x10 '"'. Ids are the inventory's (the u32 slots at [0x35ec18]) and the item table's
([0x35ec08], 0x38 bytes per item); checked against the Lost Forest's Union Plume (id 0xe7).

  tools/vp2_item_names.py --assets build_usa/extract/assets --out build_usa/bozo/item_names.txt
"""
import argparse
import os
import sys

SPECIAL = {0x0b: '-', 0x0e: ' ', 0x0f: "'", 0x10: '"', 0x14: "'"}
ITEMS = 0x2ef   # the item table's ids (mod003 bounds every item id below this)


def encode(text):
    return bytes(0x0e if c == ' ' else ord(c) - 0x1f for c in text)


def decode(raw):
    return ''.join(SPECIAL.get(b, chr(b + 0x1f) if 0x11 <= b < 0x60 else '?') for b in raw)


def names_in(data):
    """The item names of one message file, or None: the list that starts with "Long Sword\\0Rapier"."""
    at = data.find(b'\0' + encode('Long Sword') + b'\0' + encode('Rapier') + b'\0')
    if at < 0:
        return None
    parts = data[at + 1:].split(b'\0')[:ITEMS]
    if len(parts) < ITEMS or not all(len(p) <= 48 for p in parts):
        return None
    names = [decode(p) for p in parts]
    return names if names[0xe7] == 'Union Plume' else None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--assets', required=True)
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    for root, _, files in os.walk(a.assets):
        for name in sorted(files):
            if not name.endswith('.mcps'):
                continue
            path = os.path.join(root, name)
            if os.path.getsize(path) > 1 << 20:
                continue
            names = names_in(open(path, 'rb').read())
            if names:
                os.makedirs(os.path.dirname(a.out) or '.', exist_ok=True)
                with open(a.out, 'w', encoding='utf-8', newline='\n') as f:
                    f.write('\n'.join(names) + '\n')
                print(f'{len(names)} item names from {path}')
                return 0
    print('no item name list found in the assets')
    return 1


if __name__ == '__main__':
    sys.exit(main())
