"""Prepare a private original-code timer fixture, without altering load bytes."""
import argparse
from pathlib import Path
from vp2_bozo_prepare import prepare
from vp2_bozo_kernel import bindings

ap=argparse.ArgumentParser(description=__doc__)
ap.add_argument('source',type=Path)
ap.add_argument('out',type=Path)
ap.add_argument('--build',type=Path,default=Path(__file__).resolve().parents[1]/'build_usa')
args=ap.parse_args()
if not args.out.resolve().is_relative_to(args.build.resolve()):
    raise ValueError('private original-code fixtures must stay in the private build tree')
data=args.source.read_bytes()
bindings(data) # full audited USA source guards, including initializer/helper/IRQ
rows=[dict(Name=name,Start=hex(lo),End=hex(hi)) for name,lo,hi in [
    ('timer_original',0x141bb0,0x141d04),
    ('callback_pool_original',0x142e80,0x142ecc),
    ('critical_original',0x1412a0,0x1412f0)]]
fixture,_=prepare(data,rows,{})
args.out.mkdir(parents=True,exist_ok=True)
(args.out/'fixture.elf').write_bytes(fixture)
(args.out/'externals.txt').write_text('stub 0x132e40\nstub 0x136ed0\n')
