"""Compile frame-event bookkeeping after a logical parity input; never answer its GS read."""
import argparse
from pathlib import Path
from vp2_bozo_prepare import prepare
from vp2_bozo_kernel import frame_adaptations
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('source',type=Path);p.add_argument('out',type=Path);p.add_argument('--build',type=Path,required=True)
p.add_argument('--variant',choices=['original','adapted'],required=True)
a=p.parse_args()
if not a.out.resolve().is_relative_to(a.build.resolve()):raise ValueError('private build directory required')
data=a.source.read_bytes();patches=frame_adaptations(data)
start=0x11fb58 if a.variant=='original' else 0x11fb40
fixture,_=prepare(data,[dict(Name='frame_event',Start=hex(start),End='0x11fc88')],{})
a.out.mkdir(parents=True,exist_ok=True);(a.out/'fixture.elf').write_bytes(fixture)
externals=''.join(f'stub 0x{pc:x}\n' for pc in [0x114240,0x113a60,0x10d0f0,0x101060,0x1035e0,0x1195e0,0x119650])
if a.variant=='adapted':externals+='native 0x11fb50\n'
(a.out/'externals.txt').write_text(externals);(a.out/'patches.txt').write_text(patches)
