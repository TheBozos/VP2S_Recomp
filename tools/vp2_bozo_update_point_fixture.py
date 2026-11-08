"""Compile both original point-transform branches and scalar offset consumers.

This is a bounded fragment of the large object update, not a complete-owner
qualification. A fixture-only return replaces the following unrelated block.
"""
import argparse,struct
from pathlib import Path
from vp2_bozo_prepare import prepare,elf_parts
from vp2_bozo_kernel import overlay_adaptations,OBJECT_UPDATE_POINT_SITES
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('source',type=Path);p.add_argument('out',type=Path);p.add_argument('--build',type=Path,required=True);a=p.parse_args()
if not a.out.resolve().is_relative_to(a.build.resolve()):raise ValueError('private build directory required')
data=a.source.read_bytes();overlay_adaptations('mod001',data);_,segments,_=elf_parts(data)
s=next(s for s in segments if s[0]==1 and s[2]<=0x227d58<0x227e18<=s[2]+s[4]);at=s[1]+0x227e10-s[2]
actual=[pc for pc in range(0x227d58,0x227e10,4) if int.from_bytes(data[s[1]+pc-s[2]:s[1]+pc-s[2]+4],'little')>>26 in (0x12,0x36,0x3e)]
if actual!=list(OBJECT_UPDATE_POINT_SITES):raise ValueError('object update point site coverage mismatch')
changed=bytearray(data);struct.pack_into('<2I',changed,at,0x03e00008,0)
fixture,_=prepare(changed,[dict(Name='object_update_point',Start='0x227d58',End='0x227e18')],{})
a.out.mkdir(parents=True,exist_ok=True);(a.out/'fixture.elf').write_bytes(fixture)
(a.out/'externals.txt').write_text(''.join(f'native 0x{pc:x}\n' for pc in OBJECT_UPDATE_POINT_SITES))
