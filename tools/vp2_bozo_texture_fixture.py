"""Private typed texture fixtures; no console upload consumer executes."""
import argparse
import hashlib
import struct
from pathlib import Path
from vp2_bozo_prepare import prepare
from vp2_bozo_kernel import overlay_adaptations,overlay_native_sites
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('source',type=Path);p.add_argument('out',type=Path);p.add_argument('--build',type=Path,required=True)
p.add_argument('--variant',choices=['original','adapted'],required=True)
a=p.parse_args()
if not a.out.resolve().is_relative_to(a.build.resolve()):raise ValueError('private build directory required')
data=a.source.read_bytes();overlay_adaptations('mod001',data)
fixture,_=prepare(data,[dict(Name='texture_publication',Start='0x29cc30',End='0x29cf58'),
                       dict(Name='static_palette',Start='0x29cb20',End='0x29cc30'),
                       dict(Name='fixed_image',Start='0x2f36f0',End='0x2f3724'),
                       dict(Name='image_completion',Start='0x21c9a0',End='0x21ca50')],{})
patches=''.join(line+'\n' for line in overlay_adaptations('mod001',data).splitlines() if 0x29cb20<=int(line.split()[0],16)<0x29cf58)
if a.variant=='original':patches=''
sites=[0x29cc30,0x29cf30] if a.variant=='original' else [0x29cc30,0x29cd7c,0x29cf14,0x29cf30,0x29cc00]
patches+='002f3710 0c0bbce4 0c087268\n'
sites += [0x2f3714,0x21c9dc,0x21c9e0,0x21c9e8,0x21c9ec,0x21ca1c,0x21ca20,0x21ca24,0x21ca28]
a.out.mkdir(parents=True,exist_ok=True)
(a.out/'fixture.elf').write_bytes(fixture);(a.out/'patches.txt').write_text(patches)
(a.out/'externals.txt').write_text(''.join(f'native 0x{pc:x}\n' for pc in sites)+''.join(f'stub 0x{pc:x}\n' for pc in (0x100ab0,0x100b30,0x113630,0x1137f0,0x1228d0,0x137460,0x1131c0,0x133f70,0x1130f0)))
assets=a.out/'assets';assets.mkdir(exist_ok=True);manifest='VPNT1\n'
for index in (1,2):
    texels=bytes((i+index)%16 for i in range(256))
    palette=b''.join(struct.pack('<I',(i*7%256)|(index*30<<8)|(i<<16)|(128<<24)) for i in range(256))
    packed=struct.pack('<III',16,16,0x14)+texels+palette
    name='c_'+hashlib.sha1(packed).hexdigest()[:16]+'.tex';(assets/name).write_bytes(packed)
    manifest+=f'12345678 {index:x} {name}\n'
linear=b''.join(struct.pack('<I',0x80000000|i*0x010101) for i in range(256))
packed=struct.pack('<III',256,1,0)+linear+linear
name='c_'+hashlib.sha1(packed).hexdigest()[:16]+'.tex';(assets/name).write_bytes(packed)
manifest+=f'50325056 314200 {name}\n'
(assets/'manifest.txt').write_text(manifest)
