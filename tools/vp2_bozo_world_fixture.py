"""Compile object-transform CPU flow with native/independent numeric records."""
import argparse
from pathlib import Path
from vp2_bozo_prepare import prepare,elf_parts
from vp2_bozo_kernel import overlay_adaptations
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('source',type=Path);p.add_argument('out',type=Path);p.add_argument('--build',type=Path,required=True)
p.add_argument('--variant',choices=['original','adapted'],required=True)
a=p.parse_args()
if not a.out.resolve().is_relative_to(a.build.resolve()):raise ValueError('private build directory required')
data=a.source.read_bytes();overlay_adaptations('mod001',data)
lo,hi=0x248fc0,0x24926c;fixture,_=prepare(data,[dict(Name='world_transform',Start=hex(lo),End=hex(hi))],{})
_,segments,_=elf_parts(data);s=next(s for s in segments if s[0]==1 and s[2]<=lo<hi<=s[2]+s[4])
sites=[]
for pc in range(lo,hi,4):
    word=int.from_bytes(data[s[1]+pc-s[2]:s[1]+pc-s[2]+4],'little')
    if word>>26 in (0x12,0x36,0x3e):sites.append(f'native 0x{pc:x}\n')
a.out.mkdir(parents=True,exist_ok=True);(a.out/'fixture.elf').write_bytes(fixture)
(a.out/'externals.txt').write_text(''.join(sites)+'stub 0x170100\n')
