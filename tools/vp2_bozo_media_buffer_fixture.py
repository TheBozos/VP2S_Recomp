"""Original media reset CPU prefix under an explicit native transport contract."""
import argparse
from pathlib import Path
from vp2_bozo_prepare import prepare,elf_parts
from vp2_bozo_kernel import overlay_adaptations
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('source',type=Path);p.add_argument('out',type=Path);p.add_argument('--build',type=Path,required=True)
p.add_argument('--variant',choices=['original','adapted'],required=True);a=p.parse_args()
if not a.out.resolve().is_relative_to(a.build.resolve()):raise ValueError('private build directory required')
data=a.source.read_bytes();patches=overlay_adaptations('mod001',data)
fixture,_=prepare(data,[dict(Name='mpeg_create',Start='0x1ef6f8',End='0x1ef930'),dict(Name='reset_state',Start='0x1efa30',End='0x1efaf8'),
                      dict(Name='mpeg_reset',Start='0x1f0438',End='0x1f04b4'),dict(Name='mpeg_callback',Start='0x1f0510',End='0x1f0544'),dict(Name='clear_refs',Start='0x1f06c8',End='0x1f071c'),
                      dict(Name='pool_init',Start='0x1f08e8',End='0x1f08fc'),dict(Name='pool_reset',Start='0x1f0900',End='0x1f090c'),dict(Name='pool_alloc',Start='0x1f0920',End='0x1f098c'),
                      dict(Name='mpeg_stream_callback',Start='0x1f4990',End='0x1f4ac0'),
                      dict(Name='system_header',Start='0x1f1f70',End='0x1f2140'),dict(Name='pack_header',Start='0x1f2140',End='0x1f27e8'),
                      dict(Name='pes_header',Start='0x1f27e8',End='0x1f4590'),dict(Name='pss_demux',Start='0x1f4590',End='0x1f4970'),
                      dict(Name='audio_copy',Start='0x2b9f00',End='0x2ba040'),dict(Name='audio_channel1',Start='0x2bd9b0',End='0x2bdab4'),
                      dict(Name='audio_channel0',Start='0x2bdac0',End='0x2bdbc4'),
                      dict(Name='movie_constructor',Start='0x2ba600',End='0x2ba6d0'),dict(Name='media_buffer_reset',Start='0x2bbfc0',End='0x2bc1dc'),
                      dict(Name='buffer_constructor',Start='0x2bc330',End='0x2bc3f4')],{})
a.out.mkdir(parents=True,exist_ok=True);(a.out/'fixture.elf').write_bytes(fixture)
_,segments,_=elf_parts(data);segment=next(s for s in segments if s[0]==1 and s[2]<=0x313f00<0x313fa0<=s[2]+s[4])
(a.out/'streams.bin').write_bytes(data[segment[1]+0x313f00-segment[2]:segment[1]+0x313fa0-segment[2]])
lookup=next(s for s in segments if s[0]==1 and s[2]<=0x356270<0x356280<=s[2]+s[4])
(a.out/'pes.bin').write_bytes(data[lookup[1]+0x356270-lookup[2]:lookup[1]+0x356280-lookup[2]])
# Both variants omit console transport setup. Their CPU prefixes remain intact.
patches=''.join(line+'\n' for line in patches.splitlines() if line.startswith(('002bc120 ','002bc124 ','001ef864 ','001ef86c ','001efa40 ','001efae8 ','001efaec ','001f048c ','001f049c ','001f04a4 ','001f04a8 ')))
if a.variant=='original':patches+='002ba670 ac220010 00000000\n'
(a.out/'patches.txt').write_text(patches)
sites=(0x2bbfc0,0x2bc1d0,0x2ba670,0x1ef6f8,0x1ef914,0x1f0438,0x1f04a0,0x1efa30,0x1f4590,0x1f4940,0x2bdac0,0x2bd9b0) if a.variant=='adapted' else ()
(a.out/'externals.txt').write_text(''.join(f'native 0x{pc:x}\n' for pc in sites)+''.join(f'stub 0x{pc:x}\n' for pc in (0x1412a0,0x1412f0,0x1371e0,0x100ab0,0x2bd380,0x132e40,0x132c88,0x1f4ce0,0x1f4d38,0x1ef628,0x1f50a0,0x2b90a0)))
