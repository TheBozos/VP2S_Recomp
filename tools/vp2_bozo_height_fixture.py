"""Private complete CPU height-bounds builder with typed numeric reference."""
import argparse
from pathlib import Path
from vp2_bozo_prepare import prepare,elf_parts
from vp2_bozo_kernel import overlay_adaptations
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('source',type=Path);p.add_argument('out',type=Path);p.add_argument('--build',type=Path,required=True)
p.add_argument('--variant',choices=('reference','adapted'),required=True);a=p.parse_args()
if not a.out.resolve().is_relative_to(a.build.resolve()):raise ValueError('private build directory required')
data=a.source.read_bytes();overlay_adaptations('mod003',data)
fixture,_=prepare(data,[dict(Name='height_bounds',Start='0x3e5bc0',End='0x3e5d34')],{})
fixture=bytearray(fixture);_,segments,_=elf_parts(fixture)
s=next(s for s in segments if s[0]==1 and s[2]<=0x3e5bc0<0x3e5d34<=s[2]+s[4])
sites=[pc for pc in range(0x3e5bc0,0x3e5d34,4) if int.from_bytes(fixture[s[1]+pc-s[2]:s[1]+pc-s[2]+4],'little')>>26 in (0x12,0x36,0x3e)]
if sites!=[0x3e5ca4,0x3e5ca8,0x3e5cac,0x3e5cb4,0x3e5cb8,0x3e5cc0]:raise ValueError('height-bound numeric coverage mismatch')
bound=[]
for pc in sites:
    if a.variant=='reference' and pc!=0x3e5cc0:
        at=s[1]+pc-s[2];fixture[at:at+4]=bytes(4)
    else:bound.append(f'native 0x{pc:x}\n')
a.out.mkdir(parents=True,exist_ok=True)
(a.out/'fixture.elf').write_bytes(fixture);(a.out/'externals.txt').write_text(''.join(bound))
