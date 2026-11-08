"""Compile original scalar Hermite weights and complete vector result caller."""
import argparse
from pathlib import Path
from vp2_bozo_prepare import prepare,elf_parts
from vp2_bozo_kernel import overlay_adaptations,HERMITE_VECTOR_SITES
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('source',type=Path);p.add_argument('out',type=Path);p.add_argument('--build',type=Path,required=True)
p.add_argument('--variant',choices=['original','adapted'],required=True);a=p.parse_args()
if not a.out.resolve().is_relative_to(a.build.resolve()):raise ValueError('private build directory required')
data=a.source.read_bytes();overlay_adaptations('mod001',data)
fixture,_=prepare(data,[dict(Name='hermite_vector',Start='0x2a68a0',End='0x2a691c'),dict(Name='hermite_weights',Start='0x2a6980',End='0x2a69f0')],{})
fixture=bytearray(fixture);_,segments,_=elf_parts(fixture);sites=[]
for pc in HERMITE_VECTOR_SITES:
    if a.variant=='original' and pc!=0x2a68fc:
        s=next(s for s in segments if s[0]==1 and s[2]<=pc<pc+4<=s[2]+s[4]);fixture[s[1]+pc-s[2]:s[1]+pc-s[2]+4]=bytes(4)
    else:sites.append(f'native 0x{pc:x}\n')
a.out.mkdir(parents=True,exist_ok=True);(a.out/'fixture.elf').write_bytes(fixture);(a.out/'externals.txt').write_text(''.join(sites))
