"""Bound the original callback argument transforms before the unrelated call."""
import argparse,struct
from pathlib import Path
from vp2_bozo_prepare import prepare,elf_parts
from vp2_bozo_kernel import overlay_adaptations,PARTICLE_CALLBACK_TRANSFORM_SITES
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('source',type=Path);p.add_argument('out',type=Path);p.add_argument('--build',type=Path,required=True);a=p.parse_args()
if not a.out.resolve().is_relative_to(a.build.resolve()):raise ValueError('private build directory required')
data=a.source.read_bytes();overlay_adaptations('mod003',data);_,segments,_=elf_parts(data)
s=next(s for s in segments if s[0]==1 and s[2]<=0x410640<0x4107c4<=s[2]+s[4])
actual=[pc for pc in range(0x410640,0x410728,4) if int.from_bytes(data[s[1]+pc-s[2]:s[1]+pc-s[2]+4],'little')>>26 in (0x12,0x36,0x3e)]
if actual!=list(PARTICLE_CALLBACK_TRANSFORM_SITES):raise ValueError('particle callback transform coverage mismatch')
changed=bytearray(data)
# Keep the real prologue/epilogue, branch around the separately audited callback.
struct.pack_into('<2I',changed,s[1]+0x410728-s[2],0x10000023,0)
fixture,_=prepare(changed,[dict(Name='particle_callback_transform',Start='0x410640',End='0x4107c4')],{})
a.out.mkdir(parents=True,exist_ok=True);(a.out/'fixture.elf').write_bytes(fixture)
(a.out/'externals.txt').write_text(''.join(f'native 0x{pc:x}\n' for pc in (*PARTICLE_CALLBACK_TRANSFORM_SITES,0x41074c,0x410750,0x410790,0x410794,0x4107a8,0x4107b0,0x4107b4)))
