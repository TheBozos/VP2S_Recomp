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
                       dict(Name='pair_distance',Start='0x1fedc0',End='0x1fee18'),
                       dict(Name='camera_scale_first',Start='0x20f240',End='0x20f280'),
                       dict(Name='camera_scale_second',Start='0x20f280',End='0x20f2c4'),
                       dict(Name='model_parameters',Start='0x23c240',End='0x23c2d8'),
                       dict(Name='inverse_selector',Start='0x248560',End='0x248624'),
                       dict(Name='column_inverse',Start='0x216cb0',End='0x216d60'),
                       dict(Name='point_direction',Start='0x1fb6a0',End='0x1fb7b8'),
                       dict(Name='object_transform0',Start='0x1fb3f4',End='0x1fb424'),
                       dict(Name='object_transform1',Start='0x1fb5ec',End='0x1fb61c'),
                       dict(Name='object_transform2',Start='0x1fb87c',End='0x1fb8ac'),
                       dict(Name='linear_transform0',Start='0x1fb1e4',End='0x1fb20c'),
                       dict(Name='linear_transform1',Start='0x1fb2fc',End='0x1fb324'),
                       dict(Name='linear_transform2',Start='0x1fb42c',End='0x1fb454'),
                       dict(Name='linear_transform3',Start='0x1fb624',End='0x1fb64c'),
                       dict(Name='motion_update',Start='0x1fc9e0',End='0x1fcb3c'),
                       dict(Name='motion_setter',Start='0x1fcb40',End='0x1fcb54')],{})
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
for lo,hi,final,exit in ((0x20f240,0x20f280,0x20f26c,0x20f278),(0x20f280,0x20f2c4,0x20f2ac,0x20f2bc)):
    s=next(s for s in segments if s[0]==1 and s[2]<=lo<hi<=s[2]+s[4])
    at=s[1]+exit-s[2];fixture[at:at+4]=(0x03e00008).to_bytes(4,'little')
    if exit==0x20f2bc:fixture[at+4:at+8]=bytes(4)
    for pc in range(lo+8,exit if exit==0x20f2bc else hi,4):
        if pc==exit:continue
        if a.variant=='reference' and pc!=final:
            at=s[1]+pc-s[2];fixture[at:at+4]=bytes(4)
        else:sites.append(f'native 0x{pc:x}\n')
s=next(s for s in segments if s[0]==1 and s[2]<=0x23c240<0x23c2d8<=s[2]+s[4])
at=s[1]+0x23c2d0-s[2];fixture[at:at+8]=(0x03e00008).to_bytes(4,'little')+bytes(4)
for begin,end,final in ((0x23c258,0x23c26c,0x23c270),(0x23c290,0x23c2a4,0x23c2a8),(0x23c2bc,0x23c2c8,0x23c2c8)):
    for pc in (*range(begin,end,4),final):
        if a.variant=='reference' and pc!=final:
            at=s[1]+pc-s[2];fixture[at:at+4]=bytes(4)
        else:sites.append(f'native 0x{pc:x}\n')
s=next(s for s in segments if s[0]==1 and s[2]<=0x248560<0x248624<=s[2]+s[4])
for pc in (0x248570,0x248574,0x248578,0x248580,0x2485a4,0x2485a8,0x2485ac,0x2485b4):
    if a.variant=='reference' and pc!=0x2485b4:
        at=s[1]+pc-s[2];fixture[at:at+4]=bytes(4)
    else:sites.append(f'native 0x{pc:x}\n')
sites += ['native 0x216cb0\n','stub 0x216f50\n']
s=next(s for s in segments if s[0]==1 and s[2]<=0x216cb0<0x216d60<=s[2]+s[4])
for pc in (*range(0x216d34,0x216d58,4),0x216d5c):
    if a.variant=='reference' and pc!=0x216d5c:
        at=s[1]+pc-s[2];fixture[at:at+4]=bytes(4)
    else:sites.append(f'native 0x{pc:x}\n')
s=next(s for s in segments if s[0]==1 and s[2]<=0x1fb6a0<0x1fb7b8<=s[2]+s[4])
for pc in range(0x1fb768,0x1fb7a4,4):
    if a.variant=='reference' and pc!=0x1fb79c:
        at=s[1]+pc-s[2];fixture[at:at+4]=bytes(4)
    else:sites.append(f'native 0x{pc:x}\n')
sites.append('stub 0x172000\n')
for lo,final in ((0x1fb3f4,0x1fb418),(0x1fb5ec,0x1fb610),(0x1fb87c,0x1fb8a0),
                 (0x1fb1e4,0x1fb200),(0x1fb2fc,0x1fb318),(0x1fb42c,0x1fb448),(0x1fb624,0x1fb640)):
    s=next(s for s in segments if s[0]==1 and s[2]<=lo<final+12<=s[2]+s[4])
    at=s[1]+final+4-s[2];fixture[at:at+8]=(0x03e00008).to_bytes(4,'little')+bytes(4)
    for pc in range(lo,final+4,4):
        if a.variant=='reference' and pc!=final:
            at=s[1]+pc-s[2];fixture[at:at+4]=bytes(4)
        else:sites.append(f'native 0x{pc:x}\n')
# This scalar leaf has no PC-relative instructions. Relocate its unchanged
# five words into a private fixture section; production keeps its real address.
overlay=(a.source.parent/'mod003.elf').read_bytes();overlay_adaptations('mod003',overlay)
_,overlay_segments,_=elf_parts(overlay)
o=next(s for s in overlay_segments if s[0]==1 and s[2]<=0x3749f0<0x374a04<=s[2]+s[4])
s=next(s for s in segments if s[0]==1 and s[2]<=0x1fc9e0<0x1fcb54<=s[2]+s[4])
at=s[1]+0x1fcb40-s[2];start=o[1]+0x3749f0-o[2];fixture[at:at+20]=overlay[start:start+20]
for pc in (0x1fca90,0x1fca9c,0x1fcaa0,0x1fcaa4,0x1fcaa8,0x1fcaac,0x1fcac4,0x1fcad8,0x1fcadc,0x1fcafc,0x1fcb00,0x1fcb04,0x1fcb08,0x1fcb24):
    if a.variant=='reference' and pc not in (0x1fcb08,0x1fcb24):
        at=s[1]+pc-s[2];fixture[at:at+4]=bytes(4)
    else:sites.append(f'native 0x{pc:x}\n')
a.out.mkdir(parents=True,exist_ok=True);(a.out/'fixture.elf').write_bytes(fixture)
(a.out/'externals.txt').write_text(''.join(sites))
