"""Original media reset CPU prefix under an explicit native transport contract."""
import argparse
from pathlib import Path
from vp2_bozo_prepare import prepare
from vp2_bozo_kernel import overlay_adaptations
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('source',type=Path);p.add_argument('out',type=Path);p.add_argument('--build',type=Path,required=True)
p.add_argument('--variant',choices=['original','adapted'],required=True);a=p.parse_args()
if not a.out.resolve().is_relative_to(a.build.resolve()):raise ValueError('private build directory required')
data=a.source.read_bytes();patches=overlay_adaptations('mod001',data)
fixture,_=prepare(data,[dict(Name='mpeg_create',Start='0x1ef6f8',End='0x1ef930'),dict(Name='clear_refs',Start='0x1f06c8',End='0x1f071c'),
                      dict(Name='pool_init',Start='0x1f08e8',End='0x1f08fc'),dict(Name='pool_reset',Start='0x1f0900',End='0x1f090c'),dict(Name='pool_alloc',Start='0x1f0920',End='0x1f098c'),
                      dict(Name='movie_constructor',Start='0x2ba600',End='0x2ba6d0'),dict(Name='media_buffer_reset',Start='0x2bbfc0',End='0x2bc1dc'),
                      dict(Name='buffer_constructor',Start='0x2bc330',End='0x2bc3f4')],{})
a.out.mkdir(parents=True,exist_ok=True);(a.out/'fixture.elf').write_bytes(fixture)
# Both variants omit console transport setup. Their CPU prefixes remain intact.
patches=''.join(line+'\n' for line in patches.splitlines() if line.startswith(('002bc120 ','002bc124 ','001ef864 ','001ef86c ')))
if a.variant=='original':patches+='002ba670 ac220010 00000000\n'
(a.out/'patches.txt').write_text(patches)
sites=(0x2bbfc0,0x2bc1d0,0x2ba670,0x1ef6f8,0x1ef914) if a.variant=='adapted' else ()
(a.out/'externals.txt').write_text(''.join(f'native 0x{pc:x}\n' for pc in sites)+''.join(f'stub 0x{pc:x}\n' for pc in (0x1412a0,0x1412f0,0x1371e0,0x100ab0,0x2bd380,0x132e40,0x1f4ce0,0x1f4d38,0x1ef628,0x1f0438)))
