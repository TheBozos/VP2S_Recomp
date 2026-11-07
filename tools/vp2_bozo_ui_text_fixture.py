"""Compile the original game text emitter around bounded real callback observations."""
import argparse
from pathlib import Path
from vp2_bozo_prepare import prepare
from vp2_bozo_kernel import overlay_adaptations
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('source',type=Path);p.add_argument('out',type=Path);p.add_argument('--build',type=Path,required=True)
p.add_argument('--variant',choices=['original','adapted'],required=True);p.add_argument('--glyph',action='store_true');p.add_argument('--layer',action='store_true');a=p.parse_args()
if not a.out.resolve().is_relative_to(a.build.resolve()):raise ValueError('private build directory required')
data=a.source.read_bytes();overlay_adaptations('mod001',data)
function=dict(Name='ui_glyph_builder',Start='0x2e7e90',End='0x2e8678') if a.glyph else dict(Name='ui_text_emitter',Start='0x2e74f0',End='0x2e75bc')
functions=[dict(Name='ui_layer',Start='0x2ec9c0',End='0x2ec9f4'),dict(Name='ui_completion',Start='0x21c9a0',End='0x21ca50')] if a.layer else [function]
fixture,_=prepare(data,functions,{})
a.out.mkdir(parents=True,exist_ok=True);(a.out/'fixture.elf').write_bytes(fixture)
sites=((0x2e83e4,) if a.glyph else (0x2e74f0,0x2e7520,0x2e7550,0x2e757c,0x2e75a0)) if a.variant=='adapted' else ()
stubs=() if a.glyph else (0x170100,0x170110,0x2e7e90,0x2e76a0)
patches=''
if a.layer:
    # Both fixtures use native acceptance followed by the real CPU completion.
    # Reference uses the scalar adjusted-object delay slot; adapted supplies it
    # only after draw acceptance. Neither creates a console transfer job.
    patches='002ec9e0 0c0bbce4 0c087268\n'
    if a.variant=='original':patches+='002ec9e4 248400d0 24840020\n'
    for pc,word in ((0x21c9dc,0x8c220000),(0x21c9e0,0x8e0340cc),(0x21c9e8,0x00431023),(0x21c9ec,0xae0240a4),
                    (0x21ca1c,0x8cc440cc),(0x21ca20,0x8c230000),(0x21ca24,0x00641823),(0x21ca28,0xacc340a4)):
        patches+=f'{pc:08x} {word:08x} 00000000\n'
    sites=(0x2ec9e4,);stubs=(0x1130f0,)
(a.out/'patches.txt').write_text(patches)
(a.out/'externals.txt').write_text(''.join(f'native 0x{pc:x}\n' for pc in sites)+''.join(f'stub 0x{pc:x}\n' for pc in stubs))
