"""Compile source-target composition without any device or transfer consumer."""
import argparse
from pathlib import Path
from vp2_bozo_prepare import prepare
from vp2_bozo_kernel import composition_adaptations,composition_sites
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('source',type=Path);p.add_argument('out',type=Path);p.add_argument('--build',type=Path,required=True)
p.add_argument('--variant',choices=['original','adapted'],required=True)
a=p.parse_args()
if not a.out.resolve().is_relative_to(a.build.resolve()):raise ValueError('private build directory required')
data=a.source.read_bytes();patches=composition_adaptations(data)
fixture,_=prepare(data,[dict(Name='frame_composition',Start='0x114550',End='0x114dd0')],{})
a.out.mkdir(parents=True,exist_ok=True);(a.out/'fixture.elf').write_bytes(fixture)
(a.out/'patches.txt').write_text(patches)
(a.out/'externals.txt').write_text((composition_sites() if a.variant=='adapted' else 'native 0x114ab4\nnative 0x114ac0\n')+
    ''.join(f'stub 0x{pc:x}\n' for pc in (0x1010e0,0x11f5a0,0x10d0f0,0x1131c0,0x114dd0,0x122160,0x1224d0,0x122db8,0x123020,0x137460)))
