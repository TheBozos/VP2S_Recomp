"""Bound original object-facing matrix reads, scalar position builder and output."""
import argparse,struct
from pathlib import Path
from vp2_bozo_prepare import prepare,elf_parts
from vp2_bozo_kernel import overlay_adaptations
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('source',type=Path);p.add_argument('out',type=Path);p.add_argument('--build',type=Path,required=True);a=p.parse_args()
if not a.out.resolve().is_relative_to(a.build.resolve()):raise ValueError('private build directory required')
data=a.source.read_bytes();overlay_adaptations('mod001',data);_,segments,_=elf_parts(data)
s=next(s for s in segments if s[0]==1 and s[2]<=0x220a20<0x220aa0<=s[2]+s[4]);changed=bytearray(data)
struct.pack_into('<2I',changed,s[1]+0x220a98-s[2],0x03e00008,0)
fixture,_=prepare(changed,[dict(Name='object_facing',Start='0x220a20',End='0x220aa0')],{})
sites=(0x220a20,0x220a24,0x220a28,0x220a2c,0x220a38,0x220a60,0x220a68,0x220a74,0x220a8c,0x220a94)
actual=[pc for pc in range(0x220a20,0x220a98,4) if int.from_bytes(data[s[1]+pc-s[2]:s[1]+pc-s[2]+4],'little')>>26 in (0x12,0x36,0x3e)]
if actual!=list(sites):raise ValueError('object-facing source site mismatch')
a.out.mkdir(parents=True,exist_ok=True);(a.out/'fixture.elf').write_bytes(fixture)
(a.out/'externals.txt').write_text(''.join(f'native 0x{pc:x}\n' for pc in sites))
