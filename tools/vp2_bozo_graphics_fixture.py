"""Compile the audited program-install producer; no VIF/DMA consumer is executed."""
import argparse
from pathlib import Path
from vp2_bozo_prepare import prepare
from vp2_bozo_kernel import overlay_adaptations

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('source',type=Path);p.add_argument('out',type=Path)
p.add_argument('--build',type=Path,required=True)
p.add_argument('--variant',choices=['original','adapted'],required=True)
a=p.parse_args()
if not a.out.resolve().is_relative_to(a.build.resolve()):raise ValueError('private build directory required')
data=a.source.read_bytes()
patches=overlay_adaptations('mod001',data)
fixture,_=prepare(data,[dict(Name='program_install',Start='0x29e2e0',End='0x29e310'),
                       dict(Name='math_install',Start='0x29e150',End='0x29e180')],{})
a.out.mkdir(parents=True,exist_ok=True)
(a.out/'fixture.elf').write_bytes(fixture)
(a.out/'externals.txt').write_text('stub 0x201f20\n'+('native 0x29e160\n' if a.variant=='adapted' else ''))
(a.out/'patches.txt').write_text(''.join(line+'\n' for line in patches.splitlines() if line.split()[0] in ('0029e2fc','0029e160','0029e16c')))
