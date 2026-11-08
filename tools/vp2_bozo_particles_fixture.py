"""Compile the scalar mode-3 particle producer and explicit point publications."""
import argparse
from pathlib import Path
from vp2_bozo_prepare import prepare,elf_parts
from vp2_bozo_kernel import overlay_adaptations,PARTICLE_PUBLICATION_SITES
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('source',type=Path);p.add_argument('out',type=Path);p.add_argument('--build',type=Path,required=True)
p.add_argument('--variant',choices=['original','adapted'],required=True);a=p.parse_args()
if not a.out.resolve().is_relative_to(a.build.resolve()):raise ValueError('private build directory required')
data=a.source.read_bytes();patches=overlay_adaptations('mod001',data)
fixture,_=prepare(data,[dict(Name='particle_producer',Start='0x22e650',End='0x22f0d4')],{})
sites=[f'native 0x{pc:x}\n' for pc in PARTICLE_PUBLICATION_SITES]
_,segments,_=elf_parts(fixture)
for pc in range(0x22e650,0x22f0d4,4):
    s=next(s for s in segments if s[0]==1 and s[2]<=pc<pc+4<=s[2]+s[4]);word=int.from_bytes(fixture[s[1]+pc-s[2]:s[1]+pc-s[2]+4],'little')
    if word>>26 in (0x12,0x36,0x3e) and pc not in PARTICLE_PUBLICATION_SITES:sites.append(f'native 0x{pc:x}\n')
sites += [f'stub 0x{pc:x}\n' for pc in (0x2a1230,0x137d90,0x137df8,*range(0x172000,0x1720a0,0x10))]
patches=''.join(row+'\n' for row in patches.splitlines() if int(row.split()[0],16) in (0x22ee50,0x22f048))
a.out.mkdir(parents=True,exist_ok=True);(a.out/'fixture.elf').write_bytes(fixture);(a.out/'externals.txt').write_text(''.join(sites));(a.out/'patches.txt').write_text(patches)
