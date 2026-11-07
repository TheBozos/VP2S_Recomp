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
fixture,_=prepare(data,[dict(Name='button_sheet',Start='0x2ec260',End='0x2ec31c'),dict(Name='sprite_reset',Start='0x2ec320',End='0x2ec38c')],{})
a.out.mkdir(parents=True,exist_ok=True);(a.out/'fixture.elf').write_bytes(fixture)
sites=(0x2ec308,0x2ec320) if a.variant=='adapted' else ()
(a.out/'externals.txt').write_text(''.join(f'native 0x{pc:x}\n' for pc in sites)+'stub 0x2f2800\n')
assets=a.out/'assets';assets.mkdir(exist_ok=True);manifest='VPNT1\n'
for index in (0x41,0x3d):
    pixels=struct.pack('<I',0x80402000|index)*256
    packed=struct.pack('<III',16,16,0)+pixels+pixels
    name='c_'+hashlib.sha1(packed).hexdigest()[:16]+'.tex';(assets/name).write_bytes(packed)
    manifest+=f'00494853 {index:x} {name}\n'
(assets/'manifest.txt').write_text(manifest)
