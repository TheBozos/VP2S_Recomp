"""Private scalar bone loop with typed numeric oracle boundaries."""
import argparse
from pathlib import Path
from vp2_bozo_prepare import prepare,elf_parts
from vp2_bozo_kernel import overlay_adaptations,overlay_native_sites
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('source',type=Path);p.add_argument('out',type=Path);p.add_argument('--build',type=Path,required=True)
p.add_argument('--variant',choices=('reference','adapted'),required=True);a=p.parse_args()
if not a.out.resolve().is_relative_to(a.build.resolve()):raise ValueError('private build directory required')
data=a.source.read_bytes();overlay_adaptations('mod001',data)
fixture,_=prepare(data,[dict(Name='bone_loop',Start='0x2371dc',End='0x2372d8'),
                       dict(Name='pair_distance',Start='0x1fedc0',End='0x1fee18')],{})
fixture=bytearray(fixture);_,segments,_=elf_parts(fixture)
s=next(s for s in segments if s[0]==1 and s[2]<=0x2371dc<0x2372d8<=s[2]+s[4])
at=s[1]+0x2372d0-s[2];fixture[at:at+8]=(0x03e00008).to_bytes(4,'little')+bytes(4)
sites=[]
for row in overlay_native_sites('mod001',data).splitlines():
    pc=int(row.split()[1],16)
    if not 0x2371dc<=pc<0x2372d0:continue
    if a.variant=='reference' and pc not in (0x237284,0x2372bc):
        at=s[1]+pc-s[2];fixture[at:at+4]=bytes(4)
    else:sites.append(row+'\n')
s=next(s for s in segments if s[0]==1 and s[2]<=0x1fedc0<0x1fee18<=s[2]+s[4])
at=s[1]+0x1fee10-s[2];fixture[at:at+8]=(0x03e00008).to_bytes(4,'little')+bytes(4)
for pc in range(0x1fedd4,0x1fee00,4):
    if a.variant=='reference' and pc!=0x1fedfc:
        at=s[1]+pc-s[2];fixture[at:at+4]=bytes(4)
    else:sites.append(f'native 0x{pc:x}\n')
a.out.mkdir(parents=True,exist_ok=True);(a.out/'fixture.elf').write_bytes(fixture)
(a.out/'externals.txt').write_text(''.join(sites))
