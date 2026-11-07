"""Compile original CPU result consumers and complete native math wrappers.

Most references start after the original numeric result reaches the stack.
The sphere reference retains the complete CPU builder and uses independently
calculated scale/center records at its publication points. No VU instruction
or microprogram executes in either ABI/aliasing reference.
"""
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
rows=[dict(Name='matrix',Start=hex(0x217598 if a.variant=='original' else 0x217550),End='0x2175c0'),
      dict(Name='quaternion',Start=hex(0x216758 if a.variant=='original' else 0x216730),End='0x216780'),
      dict(Name='object_math',Start=hex(0x2496e8 if a.variant=='original' else 0x2495f4),End='0x249714'),
      dict(Name='object_update',Start=hex(0x24937c if a.variant=='original' else 0x249288),End='0x24938c'),
      dict(Name='quaternion_euler',Start=hex(0x2158e0 if a.variant=='original' else 0x215870),End='0x2159c0')]
rows.append(dict(Name='sphere_getter',Start='0x239ce0',End='0x239e00'))
if a.variant=='adapted':
    rows.append(dict(Name='component_product',Start='0x211da0',End='0x211dc0'))
    rows.append(dict(Name='inline_light_product',Start='0x23c17c',End='0x23c194'))
fixture,_=prepare(data,rows,{})
sites=[];_,segments,_=elf_parts(data)
if a.variant=='adapted':
    # End only this isolated four-instruction test fragment with JR/NOP.
    fixture=bytearray(fixture);_,fs,_=elf_parts(fixture)
    s=next(s for s in fs if s[0]==1 and s[2]<=0x23c18c<0x23c194<=s[2]+s[4])
    at=s[1]+0x23c18c-s[2];fixture[at:at+8]=(0x03e00008).to_bytes(4,'little')+bytes(4)
if a.variant=='original':
    # The scalar CPU reference consumes independently calculated numeric
    # records at two publication points. It executes no VU operation/call.
    fixture=bytearray(fixture);_,fs,_=elf_parts(fixture)
    segment=next(s for s in fs if s[0]==1 and s[2]<=0x239ce0<0x239e00<=s[2]+s[4])
    for pc in range(0x239ce0,0x239e00,4):
        at=segment[1]+pc-segment[2];word=int.from_bytes(fixture[at:at+4],'little')
        if word>>26 in (0x12,0x36,0x3e):fixture[at:at+4]=(0x0000482d if pc==0x239d90 else 0).to_bytes(4,'little')
    sites=['native 0x239d98\n','native 0x239dec\n']
if a.variant=='adapted':
    for row in rows:
        lo,hi=int(row['Start'],0),int(row['End'],0)
        s=next(s for s in segments if s[0]==1 and s[2]<=lo<hi<=s[2]+s[4])
        for pc in range(lo,hi,4):
            at=s[1]+pc-s[2];word=int.from_bytes(data[at:at+4],'little')
            if word>>26 in (0x12,0x36,0x3e):sites.append(f'native 0x{pc:x}\n')
a.out.mkdir(parents=True,exist_ok=True)
(a.out/'fixture.elf').write_bytes(fixture);(a.out/'externals.txt').write_text(''.join(sites)+'stub 0x12e1d8\nstub 0x12e2d8\n')
