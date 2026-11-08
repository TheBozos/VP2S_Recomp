"""Complete cached height-query CPU routine, with typed independent references."""
import argparse
from pathlib import Path
from vp2_bozo_prepare import prepare,elf_parts
from vp2_bozo_kernel import overlay_adaptations
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('source',type=Path);p.add_argument('out',type=Path);p.add_argument('--build',type=Path,required=True)
p.add_argument('--variant',choices=('reference','adapted'),required=True);a=p.parse_args()
if not a.out.resolve().is_relative_to(a.build.resolve()):raise ValueError('private build directory required')
data=a.source.read_bytes();overlay_adaptations('mod001',data)
fixture,_=prepare(data,[dict(Name='terrain_height',Start='0x252e30',End='0x253344')],{})
fixture=bytearray(fixture);_,segments,_=elf_parts(fixture)
s=next(s for s in segments if s[0]==1 and s[2]<=0x252e30<0x253344<=s[2]+s[4])
sites=[pc for pc in range(0x252e30,0x253344,4) if int.from_bytes(fixture[s[1]+pc-s[2]:s[1]+pc-s[2]+4],'little')>>26 in (0x12,0x36,0x3e)]
if sites!=[*range(0x252ec8,0x252f24,4),0x252fdc,0x252fe0,0x252fe4,0x252fec,0x252ff0,0x252ff8]:raise ValueError('terrain numeric coverage mismatch')
bound=[]
for pc in sites:
    if a.variant=='reference' and pc not in (0x252f20,0x252ff8):
        at=s[1]+pc-s[2];fixture[at:at+4]=bytes(4)
    else:bound.append(f'native 0x{pc:x}\n')
a.out.mkdir(parents=True,exist_ok=True)
(a.out/'fixture.elf').write_bytes(fixture);(a.out/'externals.txt').write_text(''.join(bound)+'stub 0x216e40\n')
