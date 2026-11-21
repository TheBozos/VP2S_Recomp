"""Index existing offline conversions; never read assets at runtime.

A key whose disc copies all convert alike is listed as `tag index name`. A key whose copies
differ (e.g. each enemy pack's own BM01:1) needs load provenance: each copy is listed as
`tag index name pack offset` (pack = the decoded part's stem, offset = its texture header),
sorted, and the port resolves it by the origin of the file the game loaded. No GS-layout
decoder is linked into the port.
"""
import argparse
import json
import hashlib
import struct
from collections import defaultdict
from pathlib import Path, PureWindowsPath
from vp2_bozo_prepare import elf_parts

STATIC_PALETTE=(0x50325056,0x314200)


def static_palette(image):
    _,segments,_=elf_parts(image);lo,hi=0x314200,0x314600
    segment=next(s for s in segments if s[0]==1 and s[2]<=lo<hi<=s[2]+s[4])
    data=image[segment[1]+lo-segment[2]:segment[1]+hi-segment[2]]
    if hashlib.sha256(data).hexdigest()!='1143517174b62b3091763659810036f9cece68582ce77724d655249c30487f79':
        raise ValueError('USA static scene palette identity mismatch')
    # Convert the fixed game's palette to native index order OFFLINE. This
    # mapping never runs in the game, nor reads a runtime upload or GS memory.
    colours=struct.unpack('<256I',data)
    linear=b''.join(struct.pack('<I',colours[((i&7)|((i&16)>>1))+16*(((i>>5)<<1)|((i>>3)&1))]) for i in range(256))
    return struct.pack('<III',256,1,0)+linear+linear


def manifest(index, variants):
    copies=defaultdict(set);places=defaultdict(list)
    for entry in index:
        if entry['tag'] is None:continue
        location=(PureWindowsPath(entry['file']).stem,entry['at'])
        name=variants.get(location)
        copies[entry['tag'],entry['index']].add(name);places[entry['tag'],entry['index']].append((location,name))
    keys={key:next(iter(names)) for key,names in copies.items() if len(names)==1 and None not in names}
    # Differing copies with every copy converted: by provenance (keys whose index fits 16 bits).
    provenance=sorted((key[0],key[1],stem,at,name) for key,names in copies.items() if len(names)>1 and None not in names and key[1]<0x10000
                      for (stem,at),name in places[key])
    return keys,provenance


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('index',type=Path);p.add_argument('textures',type=Path);p.add_argument('out',type=Path)
    p.add_argument('--module',type=Path)
    a=p.parse_args()
    variants={}
    for line in (a.textures/'variants.txt').read_text().splitlines():
        part,offset,name=line.split();variants[part,int(offset,16)]=name
    keys,provenance=manifest(json.loads(a.index.read_text()),variants)
    if a.module:
        packed=static_palette(a.module.read_bytes());name='c_'+hashlib.sha1(packed).hexdigest()[:16]+'.tex'
        (a.textures/name).write_bytes(packed)
        if STATIC_PALETTE in keys:raise ValueError('reserved native scene palette key overlaps an asset')
        keys[STATIC_PALETTE]=name
    for name in list(keys.values())+[p[4] for p in provenance]:
        if not (a.textures/name).is_file():raise ValueError('missing offline texture '+name)
    a.out.parent.mkdir(parents=True,exist_ok=True)
    a.out.write_text('VPNT1\n'+''.join(f'{tag:08x} {index:x} {name}\n' for (tag,index),name in sorted(keys.items()))+
                     ''.join(f'{tag:08x} {index:x} {name} {stem} {at:x}\n' for tag,index,stem,at,name in provenance))
    print(f'{len(keys)} unambiguous offline texture keys, {len(provenance)} copies by load provenance; keys with an unconverted copy stay absent')


if __name__=='__main__':main()
