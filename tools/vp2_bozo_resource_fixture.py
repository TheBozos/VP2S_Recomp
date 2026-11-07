"""Private original FIS resource loader and real software completion fixture."""
import argparse
from pathlib import Path
from vp2_bozo_prepare import prepare
from vp2_bozo_kernel import overlay_adaptations

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('source',type=Path);p.add_argument('out',type=Path)
p.add_argument('--build',type=Path,required=True)
p.add_argument('--kind',choices=('resource','helpers'),required=True)
p.add_argument('--variant',choices=('reference','adapted'),required=True)
a=p.parse_args()
if not a.out.resolve().is_relative_to(a.build.resolve()):raise ValueError('private build directory required')
data=a.source.read_bytes()
if a.kind=='resource':
    patches=''.join(s+'\n' for s in overlay_adaptations('mod003',data).splitlines() if 0x4479e0<=int(s.split()[0],16)<0x447f50)
    rows=[dict(Name='resource_callback',Start='0x4479e0',End='0x447a10'),
          dict(Name='resource_draw',Start='0x447a10',End='0x447a58'),
          dict(Name='resource_load',Start='0x447af0',End='0x447f50')]
    sites=(0x447af0,0x447b84,0x447d60,0x447a48)
    stubs=(0x29c880,0x21c9a0,0x100b30,0x100cb0,0x118520,0x102650,0x137460,0x112ba0)
else:
    overlay_adaptations('mod001',data)
    rows=[dict(Name='resource_metadata',Start='0x29c880',End='0x29c8e8'),
          dict(Name='resource_completion',Start='0x21c9a0',End='0x21ca50')]
    sites=();stubs=(0x1130f0,)
    # These original writes only collect physical timer profiling; the actual
    # scene pending-object checks and software notification remain compiled.
    patches=''.join(f'{pc:08x} {word:08x} 00000000\n' for pc,word in (
        (0x21c9dc,0x8c220000),(0x21c9e0,0x8e0340cc),(0x21c9e8,0x00431023),(0x21c9ec,0xae0240a4),
        (0x21ca1c,0x8cc440cc),(0x21ca20,0x8c230000),(0x21ca24,0x00641823),(0x21ca28,0xacc340a4)))
fixture,_=prepare(data,rows,{})
a.out.mkdir(parents=True,exist_ok=True)
(a.out/'fixture.elf').write_bytes(fixture)
(a.out/'patches.txt').write_text(patches)
(a.out/'externals.txt').write_text(''.join(f'native 0x{pc:x}\n' for pc in sites)+''.join(f'stub 0x{pc:x}\n' for pc in stubs))
