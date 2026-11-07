"""Private compiled debug-font initializer and CPU writer flush fixtures."""
import argparse,hashlib,struct
from pathlib import Path
from vp2_bozo_prepare import prepare
from vp2_bozo_kernel import bindings,overlay_adaptations,debug_font_adaptations
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('source',type=Path);p.add_argument('out',type=Path);p.add_argument('--build',type=Path,required=True)
p.add_argument('--kind',choices=['init','flush','defaults'],required=True);p.add_argument('--variant',choices=['original','adapted'],required=True)
a=p.parse_args()
if not a.out.resolve().is_relative_to(a.build.resolve()):raise ValueError('private build directory required')
data=a.source.read_bytes()
if a.kind=='init':
    overlay_adaptations('mod001',data);lo,hi=0x29af80,0x29b2c4
    native='native 0x29af80\nnative 0x29b2a8\n';patches='';stubs=(0x112860,0x120fa0,0x1134c0,0x120f70)
elif a.kind=='defaults':
    overlay_adaptations('mod001',data);lo,hi=0x29c990,0x29cb18
    native='native 0x29caf8\n';patches='';stubs=(0x120fa0,0x120f70)
else:
    bindings(data);lo,hi=0x120eb0,0x120f6c
    native='native 0x120f4c\nnative 0x120f58\n';patches=debug_font_adaptations(data);stubs=(0x1131c0,)
fixture,_=prepare(data,[dict(Name='font_'+a.kind,Start=hex(lo),End=hex(hi))],{})
a.out.mkdir(parents=True,exist_ok=True);(a.out/'fixture.elf').write_bytes(fixture)
(a.out/'patches.txt').write_text(patches)
(a.out/'externals.txt').write_text((native if a.variant=='adapted' or a.kind=='defaults' else '')+''.join(f'stub 0x{pc:x}\n' for pc in stubs))
assets=a.out/'assets';assets.mkdir(exist_ok=True)
linear=struct.pack('<I',0x80332211)*256
packed=struct.pack('<III',16,16,0)+linear+bytes(1024)
name='c_'+hashlib.sha1(packed).hexdigest()[:16]+'.tex';(assets/name).write_bytes(packed)
(assets/'manifest.txt').write_text(f'VPNT1\n464b4e41 0 {name}\n')
