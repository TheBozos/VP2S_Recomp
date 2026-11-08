"""Compile the complete scalar DOF colour-source builder and real completion.

Transport is excluded in both fixtures. The reference has a bounded semantic
publication at the thunk; the adapted caller validates the native image owner.
Original main-image writer helpers execute as separately compiled functions.
"""
import argparse
from pathlib import Path
from vp2_bozo_prepare import prepare,elf_parts
from vp2_bozo_kernel import overlay_adaptations
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('source',type=Path);p.add_argument('out',type=Path);p.add_argument('--build',type=Path,required=True)
p.add_argument('--variant',choices=['original','adapted'],required=True);a=p.parse_args()
if not a.out.resolve().is_relative_to(a.build.resolve()):raise ValueError('private build directory required')
data=a.source.read_bytes();patches=overlay_adaptations('mod001',data)
rows=[dict(Name='colour_copy_builder',Start='0x2dbde0',End='0x2dc6f4'),
      dict(Name='source_dimensions',Start='0x2c81d0',End='0x2c82b4'),
      dict(Name='copy_writer',Start='0x2c76d0',End='0x2c81c4'),
      dict(Name='copy_thunk',Start='0x2dc9a0',End='0x2dc9a8'),
      dict(Name='copy_completion',Start='0x2dbd90',End='0x2dbdac'),
      dict(Name='scene_completion',Start='0x21c9a0',End='0x21ca50')]
fixture,_=prepare(data,rows,{})
_,segments,_=elf_parts(data)
for row in rows:
    lo,hi=int(row['Start'],0),int(row['End'],0)
    s=next(s for s in segments if s[0]==1 and s[2]<=lo<hi<=s[2]+s[4])
    if any(int.from_bytes(data[s[1]+pc-s[2]:s[1]+pc-s[2]+4],'little')>>26 in (0x12,0x36,0x3e) for pc in range(lo,hi,4)):
        raise ValueError('unexpected vector operation in scalar colour-copy fixture')
main,_=prepare((a.source.parent/'main.elf').read_bytes(),[
    dict(Name='writer_finish',Start='0x120d80',End='0x120e20'),
    dict(Name='writer_commit',Start='0x120f70',End='0x120f94'),
    dict(Name='writer_current',Start='0x120fa0',End='0x120fd4')],{})
a.out.mkdir(parents=True,exist_ok=True);(a.out/'fixture.elf').write_bytes(fixture);(a.out/'helpers.elf').write_bytes(main)
sites=[0x2dc9a4,0x21c9dc,0x21c9e0,0x21c9e8,0x21c9ec,0x21ca1c,0x21ca20,0x21ca24,0x21ca28]
if a.variant=='adapted':sites += [0x2dbde0,0x2dbeb4,0x2dc6c8]
(a.out/'externals.txt').write_text(''.join(f'native 0x{pc:x}\n' for pc in sites)+''.join(f'stub 0x{pc:x}\n' for pc in (0x112960,0x29d9b0,0x23da20,0x29d7c0,0x1130f0)))
selected={'002dc9a0'}
(a.out/'patches.txt').write_text(''.join(line+'\n' for line in patches.splitlines() if line.split()[0] in selected))
