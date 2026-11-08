"""Compile the complete bounded animation-precache owner, without raster pacing."""
import argparse
from pathlib import Path
from vp2_bozo_prepare import prepare
from vp2_bozo_kernel import overlay_adaptations
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('source',type=Path);p.add_argument('out',type=Path);p.add_argument('--build',type=Path,required=True);a=p.parse_args()
if not a.out.resolve().is_relative_to(a.build.resolve()):raise ValueError('private build directory required')
data=a.source.read_bytes();patches=overlay_adaptations('mod003',data)
fixture,_=prepare(data,[dict(Name='animation_precache',Start='0x420ca0',End='0x4210c4')],{})
a.out.mkdir(parents=True,exist_ok=True);(a.out/'fixture.elf').write_bytes(fixture)
(a.out/'externals.txt').write_text(''.join(f'stub 0x{pc:x}\n' for pc in (0x2a8440,0x21e7b0,0x22cfd0,0x3edda0,0x2bedb0,0x170100,0x170110,0x170120,0x170130)))
selected={'00420ed8','00420ee0','00420ee4'}
(a.out/'patches.txt').write_text(''.join(line+'\n' for line in patches.splitlines() if line.split()[0] in selected))
