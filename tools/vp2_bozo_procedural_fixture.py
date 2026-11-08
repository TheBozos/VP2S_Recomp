"""Bounded scalar projection publication, retaining original transpose/copies."""
import argparse
from pathlib import Path
from vp2_bozo_prepare import prepare
from vp2_bozo_kernel import overlay_adaptations
p=argparse.ArgumentParser(description=__doc__);p.add_argument('source',type=Path);p.add_argument('out',type=Path);p.add_argument('--build',type=Path,required=True);a=p.parse_args()
if not a.out.resolve().is_relative_to(a.build.resolve()):raise ValueError('private build directory required')
data=a.source.read_bytes();overlay_adaptations('mod001',data)
fixture,_=prepare(data,[dict(Name='projection_publication',Start='0x2481d0',End='0x248284'),dict(Name='paired_point_builder',Start='0x2476b0',End='0x247ae8')],{})
a.out.mkdir(parents=True,exist_ok=True);(a.out/'fixture.elf').write_bytes(fixture)
(a.out/'externals.txt').write_text('stub 0x120fa0\nstub 0x120f70\n'+''.join(f'native 0x{pc:x}\n' for pc in (*range(0x248260,0x248270,4),0x2478c8,0x2478cc,0x2478d0,0x2478d4,0x2478d8,0x247918,0x24795c,0x247968,0x24796c,0x247974,0x247a18,0x247a1c)))
