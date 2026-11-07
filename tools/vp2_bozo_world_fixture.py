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
rows=[dict(Name='world_transform',Start='0x248fc0',End='0x24926c'),dict(Name='rigid_inverse',Start='0x216e40',End='0x216f4c')]
fixture,_=prepare(data,rows,{})
_,segments,_=elf_parts(data)
sites=[]
for row in rows:
    lo,hi=int(row['Start'],0),int(row['End'],0);s=next(s for s in segments if s[0]==1 and s[2]<=lo<hi<=s[2]+s[4])
    for pc in range(lo,hi,4):
        word=int.from_bytes(data[s[1]+pc-s[2]:s[1]+pc-s[2]+4],'little')
        if word>>26 in (0x12,0x36,0x3e):sites.append(f'native 0x{pc:x}\n')
a.out.mkdir(parents=True,exist_ok=True);(a.out/'fixture.elf').write_bytes(fixture)
(a.out/'externals.txt').write_text(''.join(sites)+'stub 0x170100\n')
