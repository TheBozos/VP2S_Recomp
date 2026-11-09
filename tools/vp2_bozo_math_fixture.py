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
rows.append(dict(Name='axis_angle',Start='0x215640',End='0x2156c8'))
# Object pair overlap (1f8aa0): bounded spill, both basis scales, sphere test.
overlap_rows=[(0x1f8b30,0x1f8b60,0x1f8b58),(0x1f8cc8,0x1f8dbc,0x1f8db4),(0x1f8dbc,0x1f8e64,0x1f8e5c)]
# Camera point projection (209aa0): the loop of two projections and the single one.
overlap_rows+=[(0x209bd0,0x209d54,0x209d4c),(0x20a00c,0x20a090,0x20a088)]
# Segment/sphere contact (1fc4b0) from its post-call label through its own return.
overlap_rows+=[(0x1fc4f0,0x1fc5a8,None)]
# Packed unit-quaternion keys (1f9d40): each block returns after its store.
overlap_rows+=[(0x1f9d74,0x1f9db0,0x1f9da8),(0x1f9df8,0x1f9e34,0x1f9e2c),(0x2a492c,0x2a49ac,0x2a49a4),(0x2a415c,0x2a4198,0x2a4190),(0x2a4228,0x2a4264,0x2a425c),(0x2a440c,0x2a44b4,0x2a44ac)]
# Collision-tree leaf tests: whole leaf functions keep their own returns.
overlap_rows+=[(0x1f6be0,0x1f6e10,None),(0x1f6e10,0x1f6ea8,None)]
# Segment against the hit box (1f6740) with its scalar clip helper 1f6670.
overlap_rows+=[(0x1f6740,0x1f6a38,None),(0x1f6670,0x1f6740,None)]
# mod001 246760 projection blocks: each span returns after its store.
overlap_rows+=[(0x246944,0x2469a8,0x2469a0),(0x2469b8,0x2469d4,0x2469cc),(0x2469d8,0x246a3c,0x246a34),(0x246a4c,0x246a68,0x246a60),(0x246e1c,0x246e94,0x246e8c)]
# mod001 1fd110 chain blocks and bone aim: each span returns after its store.
overlap_rows+=[(0x1fda08,0x1fda90,0x1fda88),(0x1fdacc,0x1fdafc,0x1fdaf4),(0x1fdb54,0x1fdb80,0x1fdb78),(0x1fdc48,0x1fdc74,0x1fdc6c),
               (0x1fdeb0,0x1fdef4,0x1fdeec),(0x1fe790,0x1fe7f4,0x1fe7ec),(0x1fe598,0x1fe68c,0x1fe684),(0x1fe68c,0x1fe6cc,0x1fe6c4)]
# mod001 1fc300 capsule pair: the placement block and the separation block.
overlap_rows+=[(0x1fc334,0x1fc3d0,0x1fc3c8),(0x1fc3f8,0x1fc484,0x1fc47c)]
# mod001 1fbd60 plane contact: the distance and projection blocks (the local transform
# 1fbdcc is the shared fragment qualified by the bones suite).
overlap_rows+=[(0x1fbd64,0x1fbdac,0x1fbda4),(0x1fbdb8,0x1fbdcc,0x1fbdc4)]
# mod001 1fbad0 range test: transform and flat length; the full length.
overlap_rows+=[(0x1fbaf8,0x1fbb60,0x1fbb58),(0x1fbb80,0x1fbba8,0x1fbba0)]
# mod001 2178c0 matrix build: the composition block returns after its four stores.
overlap_rows+=[(0x217948,0x2179c0,0x2179b8)]
# mod001 29f5e0 list box query: the node transform.
overlap_rows+=[(0x29f674,0x29f6a4,0x29f69c)]
# mod001 23e0d0 hit frame: the composition and the row product.
overlap_rows+=[(0x23e1bc,0x23e234,0x23e22c),(0x23e300,0x23e364,0x23e35c)]
# mod001 214590 plane distance: the whole function keeps its own return.
overlap_rows+=[(0x214590,0x2145c8,None)]
# mod001 20d9d0 sphere sweep: two squared differences and the row scale.
overlap_rows+=[(0x20dc20,0x20dc40,0x20dc38),(0x20dd34,0x20dd54,0x20dd4c),(0x20e49c,0x20e4d8,0x20e4d0)]
# mod001 248e40 world position: the whole function (VCALLMS 728/ae8/830, CFC2 of VI0).
overlap_rows+=[(0x248e40,0x248fc0,None)]
published=(0x1f8d1c,0x1f8d94,0x1f8e58,0x209c94,0x209d14,0x20a084,0x1fc570,0x1f9da4,0x1f9e28,0x2a4994,0x2a49a0,0x2a418c,0x2a4258,0x2a4478,0x2a44a8,0x1f6c54,0x1f6c58,0x1f6cb4,0x1f6cb8,0x1f6d10,0x1f6d14,0x1f6d68,0x1f6da8,0x1f6de0,0x1f6e74,0x1f67fc,0x1f6800,0x1f69ac,0x1f69b0,0x1f69c0,0x1f69f4,0x24699c,0x2469c8,0x246a30,0x246a5c,0x246e7c,0x1fda70,0x1fda7c,0x1fda84,0x1fdaec,0x1fdaf0,0x1fdb74,0x1fdc68,0x1fdee4,0x1fdee8,0x1fe7e8,0x1fe63c,0x1fe680,0x1fe6c0,0x1fc3c0,0x1fc3c4,0x1fc478,0x1fbda0,0x1fbdc0,0x1fbb24,0x1fbb54,0x1fbb9c,0x2179a8,0x2179ac,0x2179b0,0x2179b4,0x29f698,0x23e21c,0x23e220,0x23e224,0x23e228,0x23e358,0x2145b8,0x20dc34,0x20dd48,0x20e4c0,0x20e4c4,0x20e4c8,0x20e4cc,0x248f18,0x248f84);vi_reads=(0x248eb4,0x1f8d14,0x1f8d8c,0x209c90,0x209d10,0x20a07c)
for lo,hi,_ in overlap_rows:rows.append(dict(Name=f'vu_bounded_{lo:x}',Start=hex(lo),End=hex(hi)))
# Animated packet UVs: the original loader and its callees run unchanged in
# both variants; the test compares its scratchpad description with the native
# record rewrite (no VU or DMA execution).
for name,lo,hi in (('uv_loader',0x219620,0x21a628),('mesh_source_next',0x25cb00,0x25cb28),('mesh_packet_next',0x25cb30,0x25cb58),
                   ('mesh_source_start',0x25cc20,0x25cc80),('mesh_packet_start',0x25cc80,0x25ccf8),('uv_flags',0x29cfd0,0x29cfd8),('uv_descriptor',0x29d0d0,0x29d130),
                   ('skin_flags',0x236ba0,0x236ba8),('skin_table',0x2372f0,0x237310)):
    rows.append(dict(Name=name,Start=hex(lo),End=hex(hi)))
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
    segment=next(s for s in fs if s[0]==1 and s[2]<=0x215640<0x2156c8<=s[2]+s[4])
    for pc in (0x2156a0,0x2156a8,0x2156ac):
        at=segment[1]+pc-segment[2];fixture[at:at+4]=bytes(4)
    sites.append('native 0x2156b0\n')
fixture=bytearray(fixture);_,fs,_=elf_parts(fixture)
for lo,hi,stop in overlap_rows:
    s=next(s for s in fs if s[0]==1 and s[2]<=lo<hi<=s[2]+s[4])
    if stop is not None:at=s[1]+stop-s[2];fixture[at:at+8]=(0x03e00008).to_bytes(4,'little')+bytes(4)
    if a.variant=='original':
        # The reference executes no VU operation: COP2 words become NOPs,
        # the VI0 reads clear t1, and the publications use oracles.
        for pc in range(lo,hi if stop is None else stop,4):
            at=s[1]+pc-s[2];word=int.from_bytes(fixture[at:at+4],'little')
            if word>>26 not in (0x12,0x36,0x3e):continue
            if pc in published:sites.append(f'native 0x{pc:x}\n')
            else:fixture[at:at+4]=(0x0000482d if pc in vi_reads else 0).to_bytes(4,'little')
if a.variant=='adapted':
    for row in rows:
        lo,hi=int(row['Start'],0),int(row['End'],0)
        s=next(s for s in segments if s[0]==1 and s[2]<=lo<hi<=s[2]+s[4])
        for pc in range(lo,hi,4):
            at=s[1]+pc-s[2];word=int.from_bytes(data[at:at+4],'little')
            if word>>26 in (0x12,0x36,0x3e):sites.append(f'native 0x{pc:x}\n')
a.out.mkdir(parents=True,exist_ok=True)
(a.out/'fixture.elf').write_bytes(fixture);(a.out/'externals.txt').write_text(''.join(sites)+'stub 0x12e1d8\nstub 0x12e2d8\nstub 0x213fd0\n')
