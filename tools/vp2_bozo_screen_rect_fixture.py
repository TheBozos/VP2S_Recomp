"""Compiled solid screen producer, scalar flush and real completion; no packet consumer."""
import argparse
from pathlib import Path
from vp2_bozo_prepare import prepare
from vp2_bozo_kernel import bindings,overlay_adaptations,debug_font_adaptations
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('source',type=Path);p.add_argument('out',type=Path);p.add_argument('--build',type=Path,required=True)
p.add_argument('--kind',choices=['screen','flush','completion'],required=True)
p.add_argument('--variant',choices=['original','adapted'],required=True)
a=p.parse_args()
if not a.out.resolve().is_relative_to(a.build.resolve()):raise ValueError('private build directory required')
data=a.source.read_bytes()
if a.kind=='screen':
    patches=overlay_adaptations('mod002',data)
    patches=''.join(line+'\n' for line in patches.splitlines() if line.startswith('003721ac '))
    lo,hi=0x372180,0x3721b4;sites=(0x3721a4,0x3721b0);stubs=(0x120eb0,0x21c9a0)
elif a.kind=='flush':
    bindings(data);lo,hi=0x120eb0,0x120f6c;stubs=(0x1131c0,)
    patches=debug_font_adaptations(data) if a.variant=='adapted' else ''
    sites=(0x120f4c,0x120f58) if a.variant=='adapted' else (0x120f58,)
else:
    overlay_adaptations('mod001',data);lo,hi=0x21c9a0,0x21ca50;patches='';stubs=(0x1130f0,)
    sites=(0x21c9dc,0x21c9e0,0x21c9e8,0x21c9ec,0x21ca1c,0x21ca20,0x21ca24,0x21ca28)
fixture,_=prepare(data,[dict(Name='screen_rect_'+a.kind,Start=hex(lo),End=hex(hi))],{})
a.out.mkdir(parents=True,exist_ok=True);(a.out/'fixture.elf').write_bytes(fixture)
(a.out/'patches.txt').write_text(patches)
(a.out/'externals.txt').write_text(''.join(f'native 0x{pc:x}\n' for pc in sites)+''.join(f'stub 0x{pc:x}\n' for pc in stubs))
