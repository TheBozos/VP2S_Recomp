"""Compile the original linear-vector branches and scalar offset consumers.

A fixture-only return replaces the next unrelated block. This is bounded
branch qualification, not the full object update.
"""
import argparse,struct
from pathlib import Path
from vp2_bozo_prepare import prepare,elf_parts
from vp2_bozo_kernel import overlay_adaptations,OBJECT_LINEAR_SITES,OBJECT_CHANGE_SITES
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('source',type=Path);p.add_argument('out',type=Path);p.add_argument('--build',type=Path,required=True);a=p.parse_args()
if not a.out.resolve().is_relative_to(a.build.resolve()):raise ValueError('private build directory required')
data=a.source.read_bytes();overlay_adaptations('mod001',data);_,segments,_=elf_parts(data)
changed=bytearray(data);rows=[]
for lo,hi,sites in ((0x229148,0x229320,OBJECT_LINEAR_SITES),(0x229578,0x229668,OBJECT_CHANGE_SITES)):
    s=next(s for s in segments if s[0]==1 and s[2]<=lo<hi<=s[2]+s[4])
    actual=[pc for pc in range(lo,hi,4) if int.from_bytes(data[s[1]+pc-s[2]:s[1]+pc-s[2]+4],'little')>>26 in (0x12,0x36,0x3e)]
    if actual!=list(sites):raise ValueError('object linear-vector site coverage mismatch')
    if lo==0x229148:struct.pack_into('<2I',changed,s[1]+0x229318-s[2],0x03e00008,0)
    else:
        struct.pack_into('<I',changed,s[1]+0x229658-s[2],0)
        struct.pack_into('<2I',changed,s[1]+0x229660-s[2],0x03e00008,0)
    rows.append(dict(Name=f'update_linear_{lo:x}',Start=hex(lo),End=hex(hi)))
fixture,_=prepare(changed,rows,{})
a.out.mkdir(parents=True,exist_ok=True);(a.out/'fixture.elf').write_bytes(fixture)
(a.out/'externals.txt').write_text(''.join(f'native 0x{pc:x}\n' for pc in (*OBJECT_LINEAR_SITES,*OBJECT_CHANGE_SITES)))
