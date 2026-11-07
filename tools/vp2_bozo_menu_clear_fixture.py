"""Private whole-function menu physical-clear comparison inputs."""
import argparse
from pathlib import Path
from vp2_bozo_prepare import prepare
from vp2_bozo_kernel import overlay_adaptations
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('source',type=Path);p.add_argument('out',type=Path);p.add_argument('--build',type=Path,required=True)
a=p.parse_args()
if not a.out.resolve().is_relative_to(a.build.resolve()):raise ValueError('private build directory required')
data=a.source.read_bytes();patches=''.join(line+'\n' for line in overlay_adaptations('mod002',data).splitlines() if line.startswith('00360ef4 '))
fixture,_=prepare(data,[dict(Name='menu_clear',Start='0x360dd0',End='0x360f44')],{})
a.out.mkdir(parents=True,exist_ok=True)
(a.out/'fixture.elf').write_bytes(fixture)
(a.out/'patches.txt').write_text(patches)
(a.out/'externals.txt').write_text(''.join(f'stub 0x{pc:x}\n' for pc in
    (0x1225b8,0x100cb0,0x118520,0x132e40,0x1228d0,0x137460,0x122bf8,0x100be0)))
