"""Compile the real36-record initialization loop with only its timer-only delay retired."""
import argparse
from pathlib import Path
from vp2_bozo_prepare import prepare
from vp2_bozo_kernel import overlay_adaptations
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('source',type=Path);p.add_argument('out',type=Path);p.add_argument('--build',type=Path,required=True);a=p.parse_args()
if not a.out.resolve().is_relative_to(a.build.resolve()):raise ValueError('private build directory required')
data=a.source.read_bytes();patches=overlay_adaptations('mod003',data)
fixture,_=prepare(data,[dict(Name='record_initialization_loop',Start='0x3d7708',End='0x3d77cc')],{})
a.out.mkdir(parents=True,exist_ok=True);(a.out/'fixture.elf').write_bytes(fixture);(a.out/'patches.txt').write_text(patches)
(a.out/'externals.txt').write_text(''.join(f'stub 0x{pc:x}\n' for pc in (0x2f8770,0x137460,0x1118e0,0x10d0f0,0x101080,0x10e820,0x10e290,0x3d77cc)))
