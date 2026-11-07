"""Original scalar button-sheet assignment and reset; native assets are synthetic."""
import argparse,hashlib,struct
from pathlib import Path
from vp2_bozo_prepare import prepare
from vp2_bozo_kernel import overlay_adaptations
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('source',type=Path);p.add_argument('out',type=Path);p.add_argument('--build',type=Path,required=True)
p.add_argument('--variant',choices=['original','adapted'],required=True);a=p.parse_args()
if not a.out.resolve().is_relative_to(a.build.resolve()):raise ValueError('private build directory required')
data=a.source.read_bytes();overlay_adaptations('mod001',data)
fixture,_=prepare(data,[dict(Name='sprite_draw',Start='0x2eb3a0',End='0x2ebbdc'),dict(Name='button_sheet',Start='0x2ec260',End='0x2ec31c'),
                      dict(Name='sprite_reset',Start='0x2ec320',End='0x2ec38c'),dict(Name='text_icons',Start='0x2efe40',End='0x2f0010'),
                      dict(Name='image_element',Start='0x2f2df0',End='0x2f2fd8')],{})
a.out.mkdir(parents=True,exist_ok=True);(a.out/'fixture.elf').write_bytes(fixture)
sites=(0x2ec308,0x2ec320,0x2efe40,0x2effe8,0x2eb3a0,0x2eb5c4,0x2ebac4,0x2ebb0c,0x2ebb58,0x2ebb9c,0x2ebba8,0x2f2df0,0x2f2fb8) if a.variant=='adapted' else ()
(a.out/'externals.txt').write_text(''.join(f'native 0x{pc:x}\n' for pc in sites)+''.join(f'stub 0x{pc:x}\n' for pc in (0x2f2800,0x2144c0,0x214300)))
assets=a.out/'assets';assets.mkdir(exist_ok=True);manifest='VPNT1\n'
for index in (0x41,0x3d):
    pixels=struct.pack('<I',0x80402000|index)*256
    packed=struct.pack('<III',16,16,0)+pixels+pixels
    name='c_'+hashlib.sha1(packed).hexdigest()[:16]+'.tex';(assets/name).write_bytes(packed)
    manifest+=f'00494853 {index:x} {name}\n'
(assets/'manifest.txt').write_text(manifest)
