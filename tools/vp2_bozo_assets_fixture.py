"""Private compiled decompression fixture; all CPU instructions stay unchanged."""
import argparse
from pathlib import Path
from vp2_bozo_prepare import prepare
from vp2_bozo_kernel import bindings,asset_sites
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('source',type=Path);p.add_argument('out',type=Path);p.add_argument('--build',type=Path,required=True)
p.add_argument('--variant',choices=['original','adapted'],required=True)
a=p.parse_args()
if not a.out.resolve().is_relative_to(a.build.resolve()):raise ValueError('private build directory required')
data=a.source.read_bytes();bindings(data)
fixture,_=prepare(data,[dict(Name='decompress',Start='0x102650',End='0x1029c4'),
                       dict(Name='decompress_start',Start='0x105a30',End='0x105b78'),
                       dict(Name='decompress_step',Start='0x105940',End='0x105a24'),
                       dict(Name='heap_free',Start='0x118230',End='0x118320')],{})
a.out.mkdir(parents=True,exist_ok=True);(a.out/'fixture.elf').write_bytes(fixture)
(a.out/'externals.txt').write_text((asset_sites() if a.variant=='adapted' else '')+
    ''.join(f'stub 0x{pc:x}\n' for pc in (0x132bf0,0x132c88,0x10d0f0,0x1010c0,0x1023e0,0x101f20,0x101950,0x1013f0,0x1054d0,0x104f50,0x104a50)))
