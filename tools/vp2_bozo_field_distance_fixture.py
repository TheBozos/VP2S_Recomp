"""Compile the original scalar field-distance builder and record consumers."""
import argparse
from pathlib import Path
from vp2_bozo_prepare import prepare,elf_parts
from vp2_bozo_kernel import overlay_adaptations
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('source',type=Path);p.add_argument('out',type=Path);p.add_argument('--build',type=Path,required=True)
p.add_argument('--variant',choices=['original','adapted'],required=True);a=p.parse_args()
if not a.out.resolve().is_relative_to(a.build.resolve()):raise ValueError('private build directory required')
data=a.source.read_bytes();overlay_adaptations('mod003',data)
fixture,_=prepare(data,[dict(Name='field_distance',Start='0x378908',End='0x3789d0'),
                       dict(Name='movement_distance',Start='0x3c5d08',End='0x3c5dd0'),
                       dict(Name='camera_distance',Start='0x37b500',End='0x37b580'),
                       dict(Name='setup_distance',Start='0x381fb8',End='0x382054'),
                       dict(Name='box_sphere',Start='0x37fa30',End='0x37fb44'),
                       dict(Name='enclosing_distance',Start='0x37f95c',End='0x37f9e0'),
                       dict(Name='model_colour',Start='0x43d280',End='0x43d35c'),
                       dict(Name='field_rotation',Start='0x379228',End='0x379294'),
                       dict(Name='row_dot',Start='0x410df0',End='0x410e50'),
                       dict(Name='action_update',Start='0x4126a0',End='0x412868'),
                       dict(Name='animation_parameter',Start='0x446520',End='0x446618'),
                       dict(Name='animation_movement',Start='0x446620',End='0x446b8c')],{})
fixture=bytearray(fixture);_,segments,_=elf_parts(fixture)
s=next(s for s in segments if s[0]==1 and s[2]<=0x378908<0x3789d0<=s[2]+s[4])
sites=[]
for begin,end,result in ((0x378980,0x3789c8,0x3789a0),(0x3c5d58,0x3c5dc8,0x3c5d78),(0x37b548,0x37b578,0x37b568),(0x382020,0x38204c,0x382040),(0x37f994,0x37f9d8,0x37f9b4)):
    s=next(s for s in segments if s[0]==1 and s[2]<=begin<end+8<=s[2]+s[4])
    at=s[1]+end-s[2];fixture[at:at+8]=(0x03e00008).to_bytes(4,'little')+bytes(4)
    for pc in range(begin,result+4,4):
        if a.variant=='original' and pc!=result:
            at=s[1]+pc-s[2];fixture[at:at+4]=bytes(4)
        else:sites.append(f'native 0x{pc:x}\n')
s=next(s for s in segments if s[0]==1 and s[2]<=0x379228<0x379294<=s[2]+s[4])
at=s[1]+0x37928c-s[2];fixture[at:at+8]=(0x03e00008).to_bytes(4,'little')+bytes(4)
for pc in (0x379234,0x37923c,0x379278,0x379280,0x379284,0x379288):
    if a.variant=='original' and pc!=0x379280:
        at=s[1]+pc-s[2];fixture[at:at+4]=bytes(4)
    else:sites.append(f'native 0x{pc:x}\n')
s=next(s for s in segments if s[0]==1 and s[2]<=0x410df0<0x410e50<=s[2]+s[4])
for pc in (*range(0x410df0,0x410e48,4),0x410e4c):
    if a.variant=='original' and pc!=0x410e4c:
        at=s[1]+pc-s[2];fixture[at:at+4]=bytes(4)
    else:sites.append(f'native 0x{pc:x}\n')
s=next(s for s in segments if s[0]==1 and s[2]<=0x37fa30<0x37fb44<=s[2]+s[4])
for pc in range(0x37faa0,0x37fadc,4):
    if a.variant=='original' and pc!=0x37fad8:
        at=s[1]+pc-s[2];fixture[at:at+4]=bytes(4)
    else:sites.append(f'native 0x{pc:x}\n')
s=next(s for s in segments if s[0]==1 and s[2]<=0x43d280<0x43d35c<=s[2]+s[4])
for pc in range(0x43d2a0,0x43d2b0,4):
    if a.variant=='original' and pc!=0x43d2ac:
        at=s[1]+pc-s[2];fixture[at:at+4]=bytes(4)
    else:sites.append(f'native 0x{pc:x}\n')
for pc in range(0x41278c,0x4127b0,4):
    s=next(s for s in segments if s[0]==1 and s[2]<=pc<pc+4<=s[2]+s[4])
    at=s[1]+pc-s[2]
    if a.variant=='original' and pc!=0x4127ac:fixture[at:at+4]=bytes(4)
    else:sites.append(f'native 0x{pc:x}\n')
for pc in range(0x446724,0x446748,4):sites.append(f'native 0x{pc:x}\n')
sites += [f'stub 0x{pc:x}\n' for pc in (0x267280,0x266950,0x172000,0x172010,0x172020,0x3d0b90,0x395ec0,0x248e00,0x172030,0x2a8760)]
a.out.mkdir(parents=True,exist_ok=True);(a.out/'fixture.elf').write_bytes(fixture);(a.out/'externals.txt').write_text(''.join(sites))
