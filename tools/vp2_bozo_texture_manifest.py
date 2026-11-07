"""Index unambiguous existing offline conversions; never read assets at runtime.

Keys whose disc copies have different converted content require load provenance
and deliberately remain absent. No GS-layout decoder is linked into the port.
"""
import argparse
import json
from collections import defaultdict
from pathlib import Path, PureWindowsPath


def manifest(index, variants):
    copies=defaultdict(set)
    for entry in index:
        if entry['tag'] is None:continue
        location=(PureWindowsPath(entry['file']).stem,entry['at'])
        name=variants.get(location)
        copies[entry['tag'],entry['index']].add(name)
    return {key:next(iter(names)) for key,names in copies.items() if len(names)==1 and None not in names}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('index',type=Path);p.add_argument('textures',type=Path);p.add_argument('out',type=Path)
    a=p.parse_args()
    variants={}
    for line in (a.textures/'variants.txt').read_text().splitlines():
        part,offset,name=line.split();variants[part,int(offset,16)]=name
    keys=manifest(json.loads(a.index.read_text()),variants)
    for name in keys.values():
        if not (a.textures/name).is_file():raise ValueError('missing offline texture '+name)
    a.out.parent.mkdir(parents=True,exist_ok=True)
    a.out.write_text('VPNT1\n'+''.join(f'{tag:08x} {index:x} {name}\n' for (tag,index),name in sorted(keys.items())))
    print(f'{len(keys)} unambiguous offline texture keys; conflicting/missing copies require load provenance')


if __name__=='__main__':main()
