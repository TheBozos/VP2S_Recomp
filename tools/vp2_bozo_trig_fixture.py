"""Compile game coefficient initialization and sin/cos CPU result consumers."""
import argparse
from pathlib import Path
from vp2_bozo_prepare import prepare,elf_parts
from vp2_bozo_kernel import overlay_adaptations
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('source',type=Path);p.add_argument('out',type=Path);p.add_argument('--build',type=Path,required=True)
p.add_argument('--variant',choices=['original','adapted'],required=True)
a=p.parse_args()
if not a.out.resolve().is_relative_to(a.build.resolve()):raise ValueError('private build directory required')
data=a.source.read_bytes();overlay_adaptations('mod001',data)
lo,hi=0x213fd0,0x2140f4
fixture,_=prepare(data,[dict(Name='sine_cosine',Start=hex(lo),End=hex(hi)),dict(Name='short_cosine',Start='0x214300',End='0x2143d4'),dict(Name='coefficients',Start='0x358490',End='0x358690')],{})
_,segments,_=elf_parts(data);s=next(s for s in segments if s[0]==1 and s[2]<=lo<hi<=s[2]+s[4]);sites=[]
for pc in range(lo,hi,4):
    word=int.from_bytes(data[s[1]+pc-s[2]:s[1]+pc-s[2]+4],'little')
    if word>>26 in (0x12,0x36,0x3e):sites.append(f'native 0x{pc:x}\n')
for pc in range(0x214300,0x2143d4,4):
    word=int.from_bytes(data[s[1]+pc-s[2]:s[1]+pc-s[2]+4],'little')
    if word>>26 in (0x12,0x36,0x3e) or pc==0x214388:sites.append(f'native 0x{pc:x}\n')
a.out.mkdir(parents=True,exist_ok=True);(a.out/'fixture.elf').write_bytes(fixture);(a.out/'externals.txt').write_text(''.join(sites))
