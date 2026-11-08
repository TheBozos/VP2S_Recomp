"""Bound the original particle colour product before its lighting consumers."""
import argparse,struct
from pathlib import Path
from vp2_bozo_prepare import prepare,elf_parts
from vp2_bozo_kernel import overlay_adaptations
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('source',type=Path);p.add_argument('out',type=Path);p.add_argument('--build',type=Path,required=True);a=p.parse_args()
if not a.out.resolve().is_relative_to(a.build.resolve()):raise ValueError('private build directory required')
data=a.source.read_bytes();overlay_adaptations('mod001',data);_,segments,_=elf_parts(data)
s=next(s for s in segments if s[0]==1 and s[2]<=0x22f2f8<0x22f318<=s[2]+s[4]);changed=bytearray(data)
struct.pack_into('<2I',changed,s[1]+0x22f310-s[2],0x03e00008,0)
fixture,_=prepare(changed,[dict(Name='particle_colour_product',Start='0x22f2f8',End='0x22f318')],{})
a.out.mkdir(parents=True,exist_ok=True);(a.out/'fixture.elf').write_bytes(fixture)
(a.out/'externals.txt').write_text(''.join(f'native 0x{pc:x}\n' for pc in range(0x22f300,0x22f310,4)))
