"""Compile the entire original scalar DOF coordinate/copy writer.

All identity/rotation setup and coordinate/packet writes remain. Only the three
explicit matrix numeric publications use independent finite equations in the
reference, with no vector execution in either fixture.
"""
import argparse
from pathlib import Path
from vp2_bozo_prepare import prepare,elf_parts
from vp2_bozo_kernel import overlay_adaptations,DOF_MATRIX_SITES
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('source',type=Path);p.add_argument('out',type=Path);p.add_argument('--build',type=Path,required=True)
p.add_argument('--variant',choices=['original','adapted'],required=True);a=p.parse_args()
if not a.out.resolve().is_relative_to(a.build.resolve()):raise ValueError('private build directory required')
data=a.source.read_bytes();overlay_adaptations('mod001',data)
fixture,_=prepare(data,[dict(Name='dof_coordinate_writer',Start='0x2d7a30',End='0x2d8ee8'),
                       dict(Name='rotation',Start='0x217c60',End='0x217cc8'),
                       dict(Name='identity',Start='0x217cd0',End='0x217cf8')],{})
fixture=bytearray(fixture);_,segments,_=elf_parts(fixture);sites=[]
for pc in DOF_MATRIX_SITES:
    if a.variant=='original' and pc not in (0x2d7c64,0x2d7ce0,0x2d7d5c):
        s=next(s for s in segments if s[0]==1 and s[2]<=pc<pc+4<=s[2]+s[4]);fixture[s[1]+pc-s[2]:s[1]+pc-s[2]+4]=bytes(4)
    else:sites.append(f'native 0x{pc:x}\n')
a.out.mkdir(parents=True,exist_ok=True);(a.out/'fixture.elf').write_bytes(fixture)
(a.out/'externals.txt').write_text(''.join(sites)+'stub 0x12dbc0\nstub 0x12df60\n')
