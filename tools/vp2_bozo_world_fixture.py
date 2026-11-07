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
rows.extend([dict(Name='frame_builder',Start='0x213b50',End='0x213e9c'),dict(Name='sine_cosine',Start='0x213fd0',End='0x2140f4'),dict(Name='coefficients',Start='0x358490',End='0x358690')])
rows.append(dict(Name='matrix_angles',Start='0x216960',End='0x216a60'))
rows.append(dict(Name='camera_frustum',Start='0x207d70',End='0x2081c8'))
rows.extend([dict(Name='camera_builder',Start='0x208cd0',End='0x2091d8'),dict(Name='transpose',Start='0x216a60',End='0x216ad0'),dict(Name='guard_projection',Start='0x2176c0',End='0x2177b8'),dict(Name='projection',Start='0x2177c0',End='0x2178a8'),dict(Name='screen_projection',Start='0x217b10',End='0x217b68'),dict(Name='identity',Start='0x217cd0',End='0x217cf8')])
fixture,_=prepare(data,rows,{})
_,segments,_=elf_parts(data)
sites=[]
for row in rows:
    lo,hi=int(row['Start'],0),int(row['End'],0);s=next(s for s in segments if s[0]==1 and s[2]<=lo<hi<=s[2]+s[4])
    for pc in range(lo,hi,4):
        word=int.from_bytes(data[s[1]+pc-s[2]:s[1]+pc-s[2]+4],'little')
        if word>>26 in (0x12,0x36,0x3e):sites.append(f'native 0x{pc:x}\n')
a.out.mkdir(parents=True,exist_ok=True);(a.out/'fixture.elf').write_bytes(fixture)
(a.out/'externals.txt').write_text(''.join(sites)+'stub 0x170100\nstub 0x12e1d8\nstub 0x12e2d8\nstub 0x12e050\nstub 0x2138f0\nstub 0x170200\nstub 0x170210\n')
