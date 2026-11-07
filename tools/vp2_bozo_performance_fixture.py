"""Compile the object-update owner, observing only unused counter writes."""
import argparse
from pathlib import Path
from vp2_bozo_prepare import prepare
from vp2_bozo_kernel import overlay_adaptations
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('source',type=Path);p.add_argument('out',type=Path);p.add_argument('--build',type=Path,required=True)
p.add_argument('--variant',choices=['original','adapted'],required=True)
a=p.parse_args()
if not a.out.resolve().is_relative_to(a.build.resolve()):raise ValueError('private build directory required')
data=a.source.read_bytes();patches=overlay_adaptations('mod001',data)
fixture,_=prepare(data,[dict(Name='object_updates',Start='0x2bef20',End='0x2bf0dc')],{})
a.out.mkdir(parents=True,exist_ok=True);(a.out/'fixture.elf').write_bytes(fixture)
(a.out/'patches.txt').write_text(''.join(line+'\n' for line in patches.splitlines() if 0x2bef34<=int(line.split()[0],16)<=0x2bef44))
(a.out/'externals.txt').write_text((''.join(f'native 0x{pc:x}\n' for pc in range(0x2bef34,0x2bef48,4)) if a.variant=='original' else '')+
    ''.join(f'stub 0x{pc:x}\n' for pc in (0x2bf190,0x100af0,0x2bf2e0,0x2bf0e0,0x100be0,0x170100)))
