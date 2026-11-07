"""Private original bounded string comparison, without a library override."""
import argparse
from pathlib import Path
from vp2_bozo_prepare import prepare
from vp2_bozo_kernel import game_entries
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('source',type=Path);p.add_argument('out',type=Path);p.add_argument('--build',type=Path,required=True)
a=p.parse_args()
if not a.out.resolve().is_relative_to(a.build.resolve()):raise ValueError('private build directory required')
data=a.source.read_bytes();rows=[r for r in game_entries(data) if r['Start']=='0x134b60']
fixture,metadata=prepare(data,rows,{})
if metadata['functions']!=1:raise ValueError('bounded string fixture must contain exactly its original routine')
a.out.mkdir(parents=True,exist_ok=True);(a.out/'fixture.elf').write_bytes(fixture)
