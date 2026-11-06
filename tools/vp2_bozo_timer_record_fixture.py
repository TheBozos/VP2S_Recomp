"""Compile only the original timer record algorithm, with no hardware prefix."""
import argparse
from pathlib import Path
from vp2_bozo_prepare import prepare
from vp2_bozo_kernel import bindings
ap=argparse.ArgumentParser(description=__doc__)
ap.add_argument('source',type=Path);ap.add_argument('out',type=Path)
ap.add_argument('--build',type=Path,required=True)
a=ap.parse_args()
if not a.out.resolve().is_relative_to(a.build.resolve()):raise ValueError('private build directory required')
data=a.source.read_bytes();bindings(data)
rows=[dict(Name=name,Start=hex(lo),End=hex(hi)) for name,lo,hi in [
    ('record_transition',0x142230,0x1422f8),
    ('record_insert',0x142090,0x142114),('record_unlink',0x142118,0x142150)]]
fixture,_=prepare(data,rows,{})
a.out.mkdir(parents=True,exist_ok=True)
(a.out/'fixture.elf').write_bytes(fixture)
(a.out/'externals.txt').write_text('stub 0x1421b0\nstub 0x1422f8\nstub 0x155000\n')
