"""Private compiled debug-font initializer and CPU writer flush fixtures."""
import argparse,hashlib,struct
from pathlib import Path
from vp2_bozo_prepare import prepare
from vp2_bozo_kernel import bindings,overlay_adaptations,debug_font_adaptations
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('source',type=Path);p.add_argument('out',type=Path);p.add_argument('--build',type=Path,required=True)
p.add_argument('--kind',choices=['init','flush','defaults','empty'],required=True);p.add_argument('--variant',choices=['original','adapted'],required=True)
a=p.parse_args()
if not a.out.resolve().is_relative_to(a.build.resolve()):raise ValueError('private build directory required')
data=a.source.read_bytes()
if a.kind=='init':
    overlay_adaptations('mod001',data);lo,hi=0x29af80,0x29b2c4
    native='native 0x29af80\nnative 0x29b2a8\n';patches='';stubs=(0x112860,0x120fa0,0x1134c0,0x120f70)
elif a.kind=='defaults':
    overlay_adaptations('mod001',data);lo,hi=0x29c990,0x29cb18
    native='native 0x29caf8\n';patches='';stubs=(0x120fa0,0x120f70)
elif a.kind=='empty':
    all_patches=overlay_adaptations('mod001',data);lo,hi=0x2075a0,0x2075e0
    patches=''.join(line+'\n' for line in all_patches.splitlines() if lo<=int(line.split()[0],16)<hi)
    native='native 0x2075bc\nnative 0x2075cc\n';stubs=(0x120eb0,0x1130f0)
    for pc,word in ((0x21c9dc,0x8c220000),(0x21c9e0,0x8e0340cc),(0x21c9e8,0x00431023),(0x21c9ec,0xae0240a4),
                    (0x21ca1c,0x8cc440cc),(0x21ca20,0x8c230000),(0x21ca24,0x00641823),(0x21ca28,0xacc340a4)):
        patches+=f'{pc:08x} {word:08x} 00000000\n'
else:
    bindings(data);lo,hi=0x120eb0,0x120f6c
    native='native 0x120f4c\nnative 0x120f58\n';patches=debug_font_adaptations(data);stubs=(0x1131c0,)
rows=[dict(Name='font_'+a.kind,Start=hex(lo),End=hex(hi))]
if a.kind=='empty':rows += [dict(Name='empty_completion',Start='0x207580',End='0x20759c'),dict(Name='scene_completion',Start='0x21c9a0',End='0x21ca50')]
fixture,_=prepare(data,rows,{})
a.out.mkdir(parents=True,exist_ok=True);(a.out/'fixture.elf').write_bytes(fixture)
(a.out/'patches.txt').write_text(patches)
(a.out/'externals.txt').write_text((native if a.variant=='adapted' or a.kind in ('defaults','empty') else '')+''.join(f'stub 0x{pc:x}\n' for pc in stubs))
assets=a.out/'assets';assets.mkdir(exist_ok=True)
linear=struct.pack('<I',0x80332211)*256
packed=struct.pack('<III',16,16,0)+linear+bytes(1024)
name='c_'+hashlib.sha1(packed).hexdigest()[:16]+'.tex';(assets/name).write_bytes(packed)
(assets/'manifest.txt').write_text(f'VPNT1\n464b4e41 0 {name}\n')
