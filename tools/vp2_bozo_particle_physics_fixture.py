"""Compile the original scalar particle physics and feedback owner."""
import argparse
from pathlib import Path
from vp2_bozo_prepare import prepare,elf_parts
from vp2_bozo_kernel import overlay_adaptations
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('source',type=Path);p.add_argument('out',type=Path);p.add_argument('--build',type=Path,required=True)
p.add_argument('--variant',choices=['original','adapted'],required=True);a=p.parse_args()
if not a.out.resolve().is_relative_to(a.build.resolve()):raise ValueError('private build directory required')
data=a.source.read_bytes();overlay_adaptations('mod003',data)
fixture,_=prepare(data,[dict(Name='particle_physics',Start='0x40a8b0',End='0x40b07c'),dict(Name='particle_callback',Start='0x410640',End='0x4107c4')],{})
_,segments,_=elf_parts(fixture);sites=['native 0x40b040\n']
for pc in (*range(0x40a8b0,0x40b07c,4),*range(0x410640,0x4107c4,4)):
    s=next(s for s in segments if s[0]==1 and s[2]<=pc<pc+4<=s[2]+s[4]);word=int.from_bytes(fixture[s[1]+pc-s[2]:s[1]+pc-s[2]+4],'little')
    if word>>26 in (0x12,0x36,0x3e):sites.append(f'native 0x{pc:x}\n')
sites+=['stub 0x2262b0\n','stub 0x4107d0\n']
a.out.mkdir(parents=True,exist_ok=True);(a.out/'fixture.elf').write_bytes(fixture);(a.out/'externals.txt').write_text(''.join(sites))
