"""Compile original particle enqueue and completion; no DMA worker is admitted."""
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
data=a.source.read_bytes();patches=overlay_adaptations('mod001',data)
fixture,_=prepare(data,[dict(Name='particle_enqueue',Start='0x22f140',End='0x22f170'),
                       dict(Name='particle_completion',Start='0x22f120',End='0x22f13c'),
                       dict(Name='scene_completion',Start='0x21c9a0',End='0x21ca50')],{})
a.out.mkdir(parents=True,exist_ok=True);(a.out/'fixture.elf').write_bytes(fixture)
sites=(0x21c9dc,0x21c9e0,0x21c9e8,0x21c9ec,0x21ca1c,0x21ca20,0x21ca24,0x21ca28)
(a.out/'externals.txt').write_text('stub 0x201f20\nstub 0x1130f0\n'+''.join(f'native 0x{pc:x}\n' for pc in sites)+('native 0x22f160\n' if a.variant=='adapted' else ''))
(a.out/'patches.txt').write_text(''.join(line+'\n' for line in patches.splitlines() if line.split()[0]=='0022f15c') if a.variant=='adapted' else '')
