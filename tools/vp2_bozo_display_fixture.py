"""Compile the display selector's original CPU flow with write-only observation.

The reference observes nominated stores without executing any GS access or
providing register/status values. Native code retires only those stores.
"""
import argparse
from pathlib import Path
from vp2_bozo_prepare import prepare
from vp2_bozo_kernel import display_adaptations,startup_clear_adaptations,DISPLAY_STORES
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('source',type=Path);p.add_argument('out',type=Path);p.add_argument('--build',type=Path,required=True)
p.add_argument('--variant',choices=['original','adapted'],required=True)
a=p.parse_args()
if not a.out.resolve().is_relative_to(a.build.resolve()):raise ValueError('private build directory required')
data=a.source.read_bytes();patches=display_adaptations(data)+startup_clear_adaptations(data)
fixture,_=prepare(data,[dict(Name='display_selection',Start='0x114240',End='0x114544'),
                       dict(Name='startup_clear',Start='0x100694',End='0x100744')],{})
externals=''.join(f'stub 0x{pc:x}\n' for pc in (0x10d0f0,0x112a40,0x1131c0,0x114dd0,0x122160,
    0x100af0,0x132e40,0x1228d0,0x137460,0x122bf8,0x1010e0,0x11f4b0,0x100b30))
if a.variant=='original':externals+=''.join(f'native 0x{pc:x}\n' for pc in DISPLAY_STORES)
a.out.mkdir(parents=True,exist_ok=True)
(a.out/'fixture.elf').write_bytes(fixture);(a.out/'externals.txt').write_text(externals);(a.out/'patches.txt').write_text(patches)
