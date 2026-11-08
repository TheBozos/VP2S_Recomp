"""Compile the original query point product and real query argument setup.

The callback is bounded separately; this does not qualify the collision query.
A fixture-only return restores its seeded outer RA after the original call.
"""
import argparse,struct
from pathlib import Path
from vp2_bozo_prepare import prepare,elf_parts
from vp2_bozo_kernel import overlay_adaptations,OBJECT_QUERY_POINT_SITES
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('source',type=Path);p.add_argument('out',type=Path);p.add_argument('--build',type=Path,required=True);a=p.parse_args()
if not a.out.resolve().is_relative_to(a.build.resolve()):raise ValueError('private build directory required')
data=a.source.read_bytes();overlay_adaptations('mod001',data);_,segments,_=elf_parts(data)
s=next(s for s in segments if s[0]==1 and s[2]<=0x227f00<0x227f54<=s[2]+s[4])
actual=[pc for pc in range(0x227f00,0x227f30,4) if int.from_bytes(data[s[1]+pc-s[2]:s[1]+pc-s[2]+4],'little')>>26 in (0x12,0x36,0x3e)]
if actual!=list(OBJECT_QUERY_POINT_SITES):raise ValueError('object query point site coverage mismatch')
changed=bytearray(data);struct.pack_into('<3I',changed,s[1]+0x227f48-s[2],0xdfbf0600,0x03e00008,0)
fixture,_=prepare(changed,[dict(Name='query_point',Start='0x227f00',End='0x227f54')],{})
a.out.mkdir(parents=True,exist_ok=True);(a.out/'fixture.elf').write_bytes(fixture)
(a.out/'externals.txt').write_text(''.join(f'native 0x{pc:x}\n' for pc in OBJECT_QUERY_POINT_SITES)+'stub 0x22d5f0\n')
