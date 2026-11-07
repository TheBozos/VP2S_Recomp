"""Compile the original game text emitter around bounded real callback observations."""
import argparse
from pathlib import Path
from vp2_bozo_prepare import prepare
from vp2_bozo_kernel import overlay_adaptations
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('source',type=Path);p.add_argument('out',type=Path);p.add_argument('--build',type=Path,required=True)
p.add_argument('--variant',choices=['original','adapted'],required=True);p.add_argument('--glyph',action='store_true');a=p.parse_args()
if not a.out.resolve().is_relative_to(a.build.resolve()):raise ValueError('private build directory required')
data=a.source.read_bytes();overlay_adaptations('mod001',data)
function=dict(Name='ui_glyph_builder',Start='0x2e7e90',End='0x2e8678') if a.glyph else dict(Name='ui_text_emitter',Start='0x2e74f0',End='0x2e75bc')
fixture,_=prepare(data,[function],{})
a.out.mkdir(parents=True,exist_ok=True);(a.out/'fixture.elf').write_bytes(fixture)
sites=((0x2e83e4,) if a.glyph else (0x2e74f0,0x2e7520,0x2e7550,0x2e757c,0x2e75a0)) if a.variant=='adapted' else ()
stubs=() if a.glyph else (0x170100,0x170110,0x2e7e90,0x2e76a0)
(a.out/'externals.txt').write_text(''.join(f'native 0x{pc:x}\n' for pc in sites)+''.join(f'stub 0x{pc:x}\n' for pc in stubs))
