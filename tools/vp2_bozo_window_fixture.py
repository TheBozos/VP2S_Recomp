"""Compile the original scalar window builder; native observation never reads its packets."""
import argparse,hashlib,struct
from pathlib import Path
from vp2_bozo_prepare import prepare,elf_parts
from vp2_bozo_kernel import overlay_adaptations
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('source',type=Path);p.add_argument('out',type=Path);p.add_argument('--build',type=Path,required=True)
p.add_argument('--variant',choices=['original','adapted'],required=True);a=p.parse_args()
if not a.out.resolve().is_relative_to(a.build.resolve()):raise ValueError('private build directory required')
data=a.source.read_bytes();overlay_adaptations('mod001',data);_,segments,_=elf_parts(data)
fixture,_=prepare(data,[dict(Name='window_builder',Start='0x2e9640',End='0x2e9720'),
                      dict(Name='border_builder',Start='0x2e9ce0',End='0x2e9e8c'),
                      dict(Name='background_builder',Start='0x2e9e90',End='0x2ea348'),
                      dict(Name='quad_list',Start='0x2ead10',End='0x2eb3a0'),
                      dict(Name='selection_builder',Start='0x30ee00',End='0x30f0b8'),
                      dict(Name='image_list_builder',Start='0x30c5a0',End='0x30c668'),
                      dict(Name='scissor_builder',Start='0x2ef5b0',End='0x2ef790')],{})
a.out.mkdir(parents=True,exist_ok=True);(a.out/'fixture.elf').write_bytes(fixture)
sites=(0x2e9640,0x2e970c,0x2e9e90,0x2ea154,0x2ea338,0x2ead10,0x2eae90,0x2eb07c,0x2eb284,0x2eb2ec,0x2eb300,0x2eb320,0x2eb340,0x2eb378,
       0x30ee00,0x30ef20,0x30ef2c,0x30ef44,0x30ef78,0x30efb8,0x30eff4,0x30f028,0x30f070,0x2ef780) if a.variant=='adapted' else ()
(a.out/'externals.txt').write_text('stub 0x2e9720\nstub 0x125090\n'+''.join(f'native 0x{pc:x}\n' for pc in sites))
s=next(s for s in segments if s[0]==1 and s[2]<=0x357900<0x3579c0<=s[2]+s[4]);at=s[1]+0x357900-s[2]
(a.out/'topology.bin').write_bytes(data[at:at+0xc0])
assets=a.out/'assets';assets.mkdir(exist_ok=True)
pixels=struct.pack('<I',0x8040203d)*256;packed=struct.pack('<III',16,16,0)+pixels+pixels
name='c_'+hashlib.sha1(packed).hexdigest()[:16]+'.tex';(assets/name).write_bytes(packed)
# The camp menu backdrop's 512x256 and 512x128 four-bit parts (indices 1 and 2); the
# second half's own palette differs and must not be used.
def half(index,palette,rows):
    return struct.pack('<III',512,rows,0x14)+bytes([index])*(512*rows)+b''.join(struct.pack('<I',c) for c in palette)
top=half(1,[0,0x80102030,0x80405060]+[0]*253,256);bottom=half(2,[0,0,0xdeadbeef]+[0]*253,128)
top_name='c_'+hashlib.sha1(top).hexdigest()[:16]+'.tex';(assets/top_name).write_bytes(top)
bottom_name='c_'+hashlib.sha1(bottom).hexdigest()[:16]+'.tex';(assets/bottom_name).write_bytes(bottom)
(assets/'manifest.txt').write_text(f'VPNT1\n00494853 3d {name}\n00494853 49 {top_name}\n00494853 4a {bottom_name}\n')
