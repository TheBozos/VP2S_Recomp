"""Compile the original scalar window builder; native observation never reads its packets."""
import argparse,hashlib,struct
from pathlib import Path
from vp2_bozo_prepare import prepare,elf_parts
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('source',type=Path);p.add_argument('out',type=Path);p.add_argument('--build',type=Path,required=True)
p.add_argument('--variant',choices=['original','adapted'],required=True);a=p.parse_args()
if not a.out.resolve().is_relative_to(a.build.resolve()):raise ValueError('private build directory required')
data=a.source.read_bytes();_,segments,_=elf_parts(data)
fixture,_=prepare(data,[dict(Name='window_builder',Start='0x2e9640',End='0x2e9720'),
                      dict(Name='border_builder',Start='0x2e9ce0',End='0x2e9e8c'),
                      dict(Name='background_builder',Start='0x2e9e90',End='0x2ea348'),
                      dict(Name='quad_list',Start='0x2ead10',End='0x2eb3a0')],{})
a.out.mkdir(parents=True,exist_ok=True);(a.out/'fixture.elf').write_bytes(fixture)
sites=(0x2e9640,0x2e970c,0x2e9e90,0x2ea154,0x2ea338,0x2ead10,0x2eae90,0x2eb07c,0x2eb284,0x2eb2ec,0x2eb300,0x2eb320,0x2eb340,0x2eb378) if a.variant=='adapted' else ()
(a.out/'externals.txt').write_text('stub 0x2e9720\n'+''.join(f'native 0x{pc:x}\n' for pc in sites))
s=next(s for s in segments if s[0]==1 and s[2]<=0x357900<0x3579c0<=s[2]+s[4]);at=s[1]+0x357900-s[2]
(a.out/'topology.bin').write_bytes(data[at:at+0xc0])
assets=a.out/'assets';assets.mkdir(exist_ok=True)
pixels=struct.pack('<I',0x8040203d)*256;packed=struct.pack('<III',16,16,0)+pixels+pixels
name='c_'+hashlib.sha1(packed).hexdigest()[:16]+'.tex';(assets/name).write_bytes(packed)
(assets/'manifest.txt').write_text(f'VPNT1\n00494853 3d {name}\n')
