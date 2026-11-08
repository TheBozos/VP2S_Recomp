"""Original leafless-query CPU control/cache path with typed numeric oracles."""
import argparse
from pathlib import Path
from vp2_bozo_prepare import prepare,elf_parts
from vp2_bozo_kernel import overlay_adaptations,LEAFLESS_QUERY_SITES
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('source',type=Path);p.add_argument('out',type=Path);p.add_argument('--build',type=Path,required=True)
p.add_argument('--variant',choices=('reference','adapted'),required=True);a=p.parse_args()
if not a.out.resolve().is_relative_to(a.build.resolve()):raise ValueError('private build directory required')
data=a.source.read_bytes();overlay_adaptations('mod001',data)
fixture,_=prepare(data,[dict(Name='query_node',Start='0x1f6f40',End='0x1f7370'),dict(Name='query_root',Start='0x1f7370',End='0x1f739c')],{})
fixture=bytearray(fixture);_,segments,_=elf_parts(fixture)
s=next(s for s in segments if s[0]==1 and s[2]<=0x1f6f40<0x1f739c<=s[2]+s[4])
actual=[pc for pc in range(0x1f6f40,0x1f739c,4) if int.from_bytes(fixture[s[1]+pc-s[2]:s[1]+pc-s[2]+4],'little')>>26 in (0x12,0x36,0x3e)]
if actual!=list(LEAFLESS_QUERY_SITES):raise ValueError('query numeric coverage mismatch')
bound=[]
for pc in actual:
    if a.variant=='reference' and pc not in (0x1f70b4,0x1f717c):
        at=s[1]+pc-s[2];fixture[at:at+4]=bytes(4)
    else:bound.append(f'native 0x{pc:x}\n')
a.out.mkdir(parents=True,exist_ok=True)
(a.out/'fixture.elf').write_bytes(fixture);(a.out/'externals.txt').write_text(''.join(bound)+'stub 0x1f6be0\nstub 0x1f6e10\n')
