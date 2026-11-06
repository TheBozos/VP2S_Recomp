"""Private original/adapted game timing-prefix fixtures; unchanged ELF load bytes."""
import argparse
from pathlib import Path
from vp2_bozo_prepare import prepare
from vp2_bozo_kernel import game_adaptations

ap=argparse.ArgumentParser(description=__doc__)
ap.add_argument('source',type=Path)
ap.add_argument('out',type=Path)
ap.add_argument('--build',type=Path,required=True)
args=ap.parse_args()
if not args.out.resolve().is_relative_to(args.build.resolve()):
    raise ValueError('fixtures must stay in the private build tree')
data=args.source.read_bytes()
patches=game_adaptations(data)
fixture,_=prepare(data,[dict(Name='timing_prefix',Start='0x1002c0',End='0x100304')],{})
args.out.mkdir(parents=True,exist_ok=True)
(args.out/'fixture.elf').write_bytes(fixture)
(args.out/'externals.txt').write_text('stub 0x1211d0\n')
(args.out/'patches.txt').write_text(patches)
