"""Original media reset CPU prefix under an explicit native transport contract."""
import argparse
from pathlib import Path
from vp2_bozo_prepare import prepare
from vp2_bozo_kernel import overlay_adaptations
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('source',type=Path);p.add_argument('out',type=Path);p.add_argument('--build',type=Path,required=True)
p.add_argument('--variant',choices=['original','adapted'],required=True);a=p.parse_args()
if not a.out.resolve().is_relative_to(a.build.resolve()):raise ValueError('private build directory required')
data=a.source.read_bytes();patches=overlay_adaptations('mod001',data)
fixture,_=prepare(data,[dict(Name='media_buffer_reset',Start='0x2bbfc0',End='0x2bc1dc')],{})
a.out.mkdir(parents=True,exist_ok=True);(a.out/'fixture.elf').write_bytes(fixture)
# Both variants omit console transport setup. Their CPU prefixes remain intact.
(a.out/'patches.txt').write_text(''.join(line+'\n' for line in patches.splitlines() if line.startswith(('002bc120 ','002bc124 '))))
sites=(0x2bbfc0,0x2bc1d0) if a.variant=='adapted' else ()
(a.out/'externals.txt').write_text(''.join(f'native 0x{pc:x}\n' for pc in sites)+'stub 0x1412a0\nstub 0x1412f0\n')
