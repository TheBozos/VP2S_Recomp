"""Validate USA game math and emit original algorithms for offline checking.

The native game uses typed C++ record operations, never these reference files.
"""
import argparse
import importlib.util
import struct
import sys
import re
import hashlib
from pathlib import Path


def lower_rigid_inverse(directory,data):
    words=[0xd8a10000,0xd8a20010,0xd8a30020,0xd8640000,
           0x4bc409bc,0x4bc410bd,0x4bc4190a,0xf8640000]
    if data.count(struct.pack('<8I',*words))!=1:
        raise ValueError('Unexpected USA rigid inverse translation block')
    changed=0
    for path in directory.glob('*.cpp'):
        m=re.search(r'_0x([0-9a-f]+)$',path.stem)
        if not m or not 0x216e40<=int(m[1],16)<=0x216efc:continue
        text=path.read_text();marker='// VP2S native rigid inverse translation v1'
        if marker in text:continue
        def patch(m):
            pc=int(m[1],16)
            if not 0x216ee0<=pc<=0x216efc:return m[0]
            assert int(m[2],16)==words[(pc-0x216ee0)//4]
            body=m[3]
            if pc<0x216ef0:pattern=r'ctx->vu0_vf\[[1-4]\] = [^\n;]+;'
            elif pc<0x216efc:pattern=r'\{ __m128.*? \}'
            else:pattern=r'WRITE128\([^\n]+_mm_castps_si128\(ctx->vu0_vf\[4\]\)\);'
            found=re.search(pattern,body,re.S);assert found,(path.name,hex(pc))
            original=found[0]
            if pc==0x216efc:
                replacement=('if (vp2sRigidInverseEnabled()) { vp2sRigidInverseTranslation(rdram, ctx, '
                             'GPR_U32(ctx, 5), GPR_U32(ctx, 3)); } else { '+original+' }')
            else:replacement='if (vp2sRigidInverseReferenceEnabled()) { '+original+' }'
            return m[0][:m[0].index(body)]+body[:found.start()]+replacement+body[found.end():]
        lowered=re.sub(r'// 0x([0-9a-f]+): 0x([0-9a-f]+)[^\n]*\n(.*?)(?=\s*// 0x[0-9a-f]+:|\Z)',patch,text,flags=re.S)
        if lowered==text:continue
        include='#include "ps2_recomp_calls.h"'
        assert lowered.count(include)==1
        declarations=('\n'+marker+'\nbool vp2sRigidInverseEnabled();\nbool vp2sRigidInverseReferenceEnabled();\n'
                      'void vp2sRigidInverseTranslation(uint8_t*, R5900Context*, uint32_t, uint32_t);\n')
        path.write_text(lowered.replace(include,include+declarations));changed+=1
    print(f'Native rigid inverse: USA block validated; {changed} EE files lowered')


def lower_rigid_column(directory,data):
    # The return instruction lies between the last arithmetic operation and
    # its result store. Preserve that JR and rewrite only the delay-slot store.
    words=[0xd89f0030,0xd8bc0000,0xd8bd0010,0xd8be0020,0x4b0006c7,
           0x4bdfe1bc,0x4bdfe8bd,0x4bdff7ca,0x4bdbffd8,0x03e00008,0xf8bf0030]
    if data.count(struct.pack('<11I',*words))!=1:
        raise ValueError('Unexpected USA column rigid inverse block')
    changed=0
    for path in directory.glob('*.cpp'):
        m=re.search(r'_0x([0-9a-f]+)$',path.stem)
        if not m or not 0x216cb0<=int(m[1],16)<=0x216d5c:continue
        text=path.read_text();marker='// VP2S native column rigid inverse v1'
        if marker in text:continue
        def patch(m):
            pc=int(m[1],16)
            if not 0x216d34<=pc<=0x216d5c:return m[0]
            index=(pc-0x216d34)//4;assert int(m[2],16)==words[index]
            if index==9:return m[0]
            body=m[3]
            if index<4:pattern=r'ctx->vu0_vf\[\d+\] = [^\n;]+;'
            elif index<9:pattern=r'\{ __m128.*? \}'
            else:pattern=r'WRITE128\([^\n]+_mm_castps_si128\(ctx->vu0_vf\[31\]\)\);'
            found=re.search(pattern,body,re.S);assert found,(path.name,hex(pc))
            original=found[0]
            if index==10:
                replacement=('if (vp2sRigidInverseEnabled()) { vp2sRigidInverseColumnTranslation(rdram, ctx, '
                             'GPR_U32(ctx, 5), ADD32(GPR_U32(ctx, 4), 48u), ADD32(GPR_U32(ctx, 5), 48u)); } else { '+original+' }')
            else:replacement='if (vp2sRigidInverseReferenceEnabled()) { '+original+' }'
            return m[0][:m[0].index(body)]+body[:found.start()]+replacement+body[found.end():]
        lowered=re.sub(r'// 0x([0-9a-f]+): 0x([0-9a-f]+)[^\n]*\n(.*?)(?=\s*// 0x[0-9a-f]+:|\Z)',patch,text,flags=re.S)
        if lowered==text:continue
        include='#include "ps2_recomp_calls.h"';assert lowered.count(include)==1
        declarations=('\n'+marker+'\nbool vp2sRigidInverseEnabled();\nbool vp2sRigidInverseReferenceEnabled();\n'
                      'void vp2sRigidInverseColumnTranslation(uint8_t*, R5900Context*, uint32_t, uint32_t, uint32_t);\n')
        path.write_text(lowered.replace(include,include+declarations));changed+=1
    print(f'Native column rigid inverse: USA block validated; {changed} EE files lowered')


def lower_projection_products(directory,data):
    words=[0xd8a10000,0xd8a20010,0xd8a30020,0xd8a40030,
           0xd8650000,0xd8660010,0xd8670020,0xd8680030]
    for column in range(1,5):
        words.extend([0x4be029bc|(column<<16),0x4be030bd|(column<<16),
                      0x4be038be|(column<<16),0x4be0400b|(column<<16)|(column<<6)])
    words.extend([0xfa010000,0xfa020010,0xfa030020,0xfa040030])
    if data.count(struct.pack('<28I',*words))!=2:
        raise ValueError('Unexpected USA projection matrix products')
    changed=0
    for path in directory.glob('*.cpp'):
        m=re.search(r'_0x([0-9a-f]+)$',path.stem)
        if not m or not 0x2178b0<=int(m[1],16)<=0x217adc:continue
        text=path.read_text();marker='// VP2S native projection products v1'
        if marker in text:continue
        def patch(m):
            pc=int(m[1],16)
            start=next((p for p in (0x217948,0x217a70) if p<=pc<=p+108),None)
            if start is None:return m[0]
            index=(pc-start)//4;assert int(m[2],16)==words[index]
            body=m[3]
            if index<8:pattern=r'ctx->vu0_vf\[[1-8]\] = [^\n;]+;'
            elif index<24:pattern=r'\{ __m128.*? \}'
            else:pattern=r'WRITE128\([^\n]+_mm_castps_si128\(ctx->vu0_vf\[[1-4]\]\)\);'
            found=re.search(pattern,body,re.S);assert found,(path.name,hex(pc))
            original=found[0]
            if index==24:
                replacement=('if (vp2sProjectionProductEnabled()) { vp2sProjectionProduct(rdram, ctx, '
                             f'GPR_U32(ctx, 5), GPR_U32(ctx, 3), GPR_U32(ctx, 16), 0x{pc:x}u); }} else {{ '+original+' }')
            else:replacement='if (vp2sProjectionProductReferenceEnabled()) { '+original+' }'
            return m[0][:m[0].index(body)]+body[:found.start()]+replacement+body[found.end():]
        lowered=re.sub(r'// 0x([0-9a-f]+): 0x([0-9a-f]+)[^\n]*\n(.*?)(?=\s*// 0x[0-9a-f]+:|\Z)',patch,text,flags=re.S)
        if lowered==text:continue
        include='#include "ps2_recomp_calls.h"';assert lowered.count(include)==1
        declarations=('\n'+marker+'\nbool vp2sProjectionProductEnabled();\nbool vp2sProjectionProductReferenceEnabled();\n'
                      'void vp2sProjectionProduct(uint8_t*, R5900Context*, uint32_t, uint32_t, uint32_t, uint32_t);\n')
        path.write_text(lowered.replace(include,include+declarations));changed+=1
    print(f'Native projection products: two USA blocks validated; {changed} EE files lowered')


def lower_transposed_rigid(directory,data):
    words=[0xd8810000,0xd8820010,0xd8830020,0xd8c40000,0x4bc409bc,0x4bc410bd,0x4bc4190a,
           0x78880000,0x78890010,0x788a0020,0x788b0030,0x71286488,0x71286ca8,0x716a7488,0x716a7ca8,
           0x71cc4389,0x718e4ba9,0x71ed5389,0x71af5ba9,0x7ca80000,0x7ca90010,0x7caa0020,0x7cab0030,
           0x27a30000,0xf8640000]
    if data.count(struct.pack('<25I',*words))!=1:
        raise ValueError('Unexpected USA transposed rigid inverse block')
    changed=0
    for path in directory.glob('*.cpp'):
        m=re.search(r'_0x([0-9a-f]+)$',path.stem)
        if not m or not 0x216d60<=int(m[1],16)<=0x216e04:continue
        text=path.read_text();marker='// VP2S native transposed rigid inverse v1'
        if marker in text:continue
        def patch(m):
            pc=int(m[1],16)
            if not 0x216da4<=pc<=0x216e04:return m[0]
            index=(pc-0x216da4)//4;assert int(m[2],16)==words[index]
            if 7<=index<24:return m[0] # retain the integer transpose and pointers
            body=m[3]
            if index<4:pattern=r'ctx->vu0_vf\[[1-4]\] = [^\n;]+;'
            elif index<7:pattern=r'\{ __m128.*? \}'
            else:pattern=r'WRITE128\([^\n]+_mm_castps_si128\(ctx->vu0_vf\[4\]\)\);'
            found=re.search(pattern,body,re.S);assert found,(path.name,hex(pc))
            original=found[0]
            if index==24:
                replacement=('if (vp2sRigidInverseEnabled()) { vp2sTransposedRigidInverseTranslation(rdram, ctx, '
                             'GPR_U32(ctx, 5), GPR_U32(ctx, 3)); } else { '+original+' }')
            else:replacement='if (vp2sRigidInverseReferenceEnabled()) { '+original+' }'
            return m[0][:m[0].index(body)]+body[:found.start()]+replacement+body[found.end():]
        lowered=re.sub(r'// 0x([0-9a-f]+): 0x([0-9a-f]+)[^\n]*\n(.*?)(?=\s*// 0x[0-9a-f]+:|\Z)',patch,text,flags=re.S)
        if lowered==text:continue
        include='#include "ps2_recomp_calls.h"';assert lowered.count(include)==1
        declarations=('\n'+marker+'\nbool vp2sRigidInverseEnabled();\nbool vp2sRigidInverseReferenceEnabled();\n'
                      'void vp2sTransposedRigidInverseTranslation(uint8_t*, R5900Context*, uint32_t, uint32_t);\n')
        path.write_text(lowered.replace(include,include+declarations));changed+=1
    print(f'Native transposed rigid inverse: USA block validated; {changed} EE files lowered')


def lower_offset_dot(directory,data):
    words=[0xd8900000,0xd8b10000,0xd8d20000,0x4bd284ec,0x4bc007c3,
           0x4bd19d2a,0x4b14a03d,0x4b14fc0a,0xf8500000]
    if data.count(struct.pack('<9I',*words))!=1:
        raise ValueError('Unexpected USA offset dot block')
    changed=0
    for path in directory.glob('*.cpp'):
        m=re.search(r'_0x([0-9a-f]+)$',path.stem)
        if not m or not 0x214590<=int(m[1],16)<=0x2145b8:continue
        text=path.read_text();marker='// VP2S native offset dot v1'
        if marker in text:continue
        def patch(m):
            pc=int(m[1],16)
            if not 0x214598<=pc<=0x2145b8:return m[0]
            index=(pc-0x214598)//4;assert int(m[2],16)==words[index]
            body=m[3]
            if index<3:pattern=r'ctx->vu0_vf\[\d+\] = [^\n;]+;'
            elif index<8:pattern=r'\{ __m128.*? \}'
            else:pattern=r'WRITE128\([^\n]+_mm_castps_si128\(ctx->vu0_vf\[16\]\)\);'
            found=re.search(pattern,body,re.S);assert found,(path.name,hex(pc))
            original=found[0]
            if index==8:
                replacement=('if (vp2sOffsetDotEnabled()) { vp2sOffsetDotRecord(rdram, ctx, GPR_U32(ctx, 4), '
                             'GPR_U32(ctx, 5), GPR_U32(ctx, 6), GPR_U32(ctx, 2)); } else { '+original+' }')
            else:replacement='if (vp2sOffsetDotReferenceEnabled()) { '+original+' }'
            return m[0][:m[0].index(body)]+body[:found.start()]+replacement+body[found.end():]
        lowered=re.sub(r'// 0x([0-9a-f]+): 0x([0-9a-f]+)[^\n]*\n(.*?)(?=\s*// 0x[0-9a-f]+:|\Z)',patch,text,flags=re.S)
        if lowered==text:continue
        include='#include "ps2_recomp_calls.h"';assert lowered.count(include)==1
        declarations=('\n'+marker+'\nbool vp2sOffsetDotEnabled();\nbool vp2sOffsetDotReferenceEnabled();\n'
                      'void vp2sOffsetDotRecord(uint8_t*, R5900Context*, uint32_t, uint32_t, uint32_t, uint32_t);\n')
        path.write_text(lowered.replace(include,include+declarations));changed+=1
    print(f'Native offset dot: USA block validated; {changed} EE files lowered')


def lower_linear_vectors(directory,data):
    words=[0xd8840000,0xd8a10000,0xd8a20010,0xd8a30020,
           0x4be409bc,0x4be410bd,0x4be4190a,0xf8640000]
    if data.count(struct.pack('<8I',*words))!=4:
        raise ValueError('Unexpected USA object linear vector blocks')
    ranges=[(0x1fb120,0x1fb240,0x1fb1e4),(0x1fb240,0x1fb330,0x1fb2fc),
            (0x1fb330,0x1fb530,0x1fb42c),(0x1fb530,0x1fb660,0x1fb624)]
    changed=0
    for path in directory.glob('*.cpp'):
        match=re.search(r'_0x([0-9a-f]+)$',path.stem)
        if not match:continue
        block=next((start for lo,hi,start in ranges if lo<=int(match[1],16)<hi),None)
        if block is None:continue
        text=path.read_text();marker='// VP2S native object linear vector v1'
        if marker in text:continue
        def patch(m):
            pc=int(m[1],16)
            if not block<=pc<=block+28:return m[0]
            index=(pc-block)//4;assert int(m[2],16)==words[index]
            body=m[3]
            pattern=(r'ctx->vu0_vf\[[1-4]\] = [^\n;]+;' if index<4 else
                     r'\{ __m128.*? \}' if index<7 else
                     r'WRITE128\([^\n]+_mm_castps_si128\(ctx->vu0_vf\[4\]\)\);')
            found=re.search(pattern,body,re.S);assert found,(path.name,hex(pc))
            original=found[0]
            if index==7:
                replacement=('if (vp2sLinearVectorEnabled()) { vp2sLinearVectorProduct(rdram, ctx, '
                             f'GPR_U32(ctx, 5), GPR_U32(ctx, 4), GPR_U32(ctx, 3), 0x{pc:x}u); }} else {{ '+original+' }')
            else:replacement='if (vp2sLinearVectorReferenceEnabled()) { '+original+' }'
            return m[0][:m[0].index(body)]+body[:found.start()]+replacement+body[found.end():]
        lowered=re.sub(r'// 0x([0-9a-f]+): 0x([0-9a-f]+)[^\n]*\n(.*?)(?=\s*// 0x[0-9a-f]+:|\Z)',patch,text,flags=re.S)
        if lowered==text:continue
        include='#include "ps2_recomp_calls.h"';assert lowered.count(include)==1
        declarations=('\n'+marker+'\nbool vp2sLinearVectorEnabled();\nbool vp2sLinearVectorReferenceEnabled();\n'
                      'void vp2sLinearVectorProduct(uint8_t*, R5900Context*, uint32_t, uint32_t, uint32_t, uint32_t);\n')
        path.write_text(lowered.replace(include,include+declarations));changed+=1
    print(f'Native object linear vectors: four USA blocks validated; {changed} EE files lowered')


def lower_matrix_vectors(directory,data):
    words=[0xd8850000,0xd8a10000,0xd8a20010,0xd8a30020,0xd8a40030,
           0x4be509bc,0x4be510bd,0x4be518be,0x4be5214b,0xf8650000]
    if data.count(struct.pack('<10I',*words))!=3:
        raise ValueError('Unexpected USA object matrix vector blocks')
    ranges=[(0x1fb330,0x1fb460,0x1fb3f4),(0x1fb530,0x1fb658,0x1fb5ec),
            (0x1fb7c0,0x1fb8b8,0x1fb87c)]
    changed=0
    for path in directory.glob('*.cpp'):
        match=re.search(r'_0x([0-9a-f]+)$',path.stem)
        if not match:continue
        block=next((start for lo,hi,start in ranges if lo<=int(match[1],16)<hi),None)
        if block is None:continue
        text=path.read_text();marker='// VP2S native object matrix vector v1'
        if marker in text:continue
        def patch(m):
            pc=int(m[1],16)
            if not block<=pc<=block+36:return m[0]
            index=(pc-block)//4;assert int(m[2],16)==words[index]
            body=m[3]
            pattern=(r'ctx->vu0_vf\[[1-5]\] = [^\n;]+;' if index<5 else
                     r'\{ __m128.*? \}' if index<9 else
                     r'WRITE128\([^\n]+_mm_castps_si128\(ctx->vu0_vf\[5\]\)\);')
            found=re.search(pattern,body,re.S);assert found,(path.name,hex(pc))
            original=found[0]
            if index==9:
                replacement=('if (vp2sMatrixVectorEnabled()) { vp2sMatrixVectorProduct(rdram, ctx, '
                             f'GPR_U32(ctx, 5), GPR_U32(ctx, 4), GPR_U32(ctx, 3), 0x{pc:x}u); }} else {{ '+original+' }')
            else:replacement='if (vp2sMatrixVectorReferenceEnabled()) { '+original+' }'
            return m[0][:m[0].index(body)]+body[:found.start()]+replacement+body[found.end():]
        lowered=re.sub(r'// 0x([0-9a-f]+): 0x([0-9a-f]+)[^\n]*\n(.*?)(?=\s*// 0x[0-9a-f]+:|\Z)',patch,text,flags=re.S)
        if lowered==text:continue
        include='#include "ps2_recomp_calls.h"';assert lowered.count(include)==1
        declarations=('\n'+marker+'\nbool vp2sMatrixVectorEnabled();\nbool vp2sMatrixVectorReferenceEnabled();\n'
                      'void vp2sMatrixVectorProduct(uint8_t*, R5900Context*, uint32_t, uint32_t, uint32_t, uint32_t);\n')
        path.write_text(lowered.replace(include,include+declarations));changed+=1
    print(f'Native object matrix vectors: three USA blocks validated; {changed} EE files lowered')


def lower_point_direction(directory,data):
    words=[0xd8c10000,0xd8c20010,0xd8c30020,0xd8c40030,0xd8a50000,0xd8860000,
           0x4be509bc,0x4be510bd,0x4be518be,0x4be5214b,0x4be609bc,0x4be610bd,
           0x4be6198a,0xf8a50000,0xf8860000]
    if data.count(struct.pack('<15I',*words))!=1:
        raise ValueError('Unexpected USA paired point direction block')
    changed=0
    for path in directory.glob('*.cpp'):
        match=re.search(r'_0x([0-9a-f]+)$',path.stem)
        if not match or not 0x1fb6a0<=int(match[1],16)<=0x1fb7a0:continue
        text=path.read_text();marker='// VP2S native paired point direction v1'
        if marker in text:continue
        def patch(m):
            pc=int(m[1],16)
            if not 0x1fb768<=pc<=0x1fb7a0:return m[0]
            index=(pc-0x1fb768)//4;assert int(m[2],16)==words[index]
            body=m[3]
            pattern=(r'ctx->vu0_vf\[[1-6]\] = [^\n;]+;' if index<6 else
                     r'\{ __m128.*? \}' if index<13 else
                     r'WRITE128\([^\n]+_mm_castps_si128\(ctx->vu0_vf\[[56]\]\)\);')
            found=re.search(pattern,body,re.S);assert found,(path.name,hex(pc))
            original=found[0]
            if index==13:
                replacement=('if (vp2sPointDirectionEnabled()) { vp2sTransformPointDirection(rdram, ctx, '
                             'GPR_U32(ctx, 6), GPR_U32(ctx, 5), GPR_U32(ctx, 4)); } else { '+original+' }')
            else:replacement='if (vp2sPointDirectionReferenceEnabled()) { '+original+' }'
            return m[0][:m[0].index(body)]+body[:found.start()]+replacement+body[found.end():]
        lowered=re.sub(r'// 0x([0-9a-f]+): 0x([0-9a-f]+)[^\n]*\n(.*?)(?=\s*// 0x[0-9a-f]+:|\Z)',patch,text,flags=re.S)
        if lowered==text:continue
        include='#include "ps2_recomp_calls.h"';assert lowered.count(include)==1
        declarations=('\n'+marker+'\nbool vp2sPointDirectionEnabled();\nbool vp2sPointDirectionReferenceEnabled();\n'
                      'void vp2sTransformPointDirection(uint8_t*, R5900Context*, uint32_t, uint32_t, uint32_t);\n')
        path.write_text(lowered.replace(include,include+declarations));changed+=1
    print(f'Native paired point direction: USA block validated; {changed} EE files lowered')


def lower_collision_point(directory,data):
    words=[0xd8a50000,0xd8610000,0xd8620010,0xd8630020,0xd8640030,
           0x4be509bc,0x4be510bd,0x4be518be,0x4be5214b,0xf8450000]
    # A second byte-identical block at 0x1FBE6C belongs to a different helper;
    # only the audited 0x1FBDCC site is lowered in this increment.
    if data.count(struct.pack('<10I',*words))!=2:
        raise ValueError('Unexpected USA collision point transform')
    changed=0
    for path in directory.glob('*.cpp'):
        match=re.search(r'_0x([0-9a-f]+)$',path.stem)
        if not match or not 0x1fbd60<=int(match[1],16)<=0x1fbdf0:continue
        text=path.read_text();marker='// VP2S native collision point transform v1'
        if marker in text:continue
        def patch(m):
            pc=int(m[1],16)
            if not 0x1fbdcc<=pc<=0x1fbdf0:return m[0]
            index=(pc-0x1fbdcc)//4;assert int(m[2],16)==words[index]
            body=m[3]
            pattern=(r'ctx->vu0_vf\[[1-5]\] = [^\n;]+;' if index<5 else
                     r'\{ __m128.*? \}' if index<9 else
                     r'WRITE128\([^\n]+_mm_castps_si128\(ctx->vu0_vf\[5\]\)\);')
            found=re.search(pattern,body,re.S);assert found,(path.name,hex(pc))
            original=found[0]
            if index==9:
                replacement=('if (vp2sMatrixVectorEnabled()) { vp2sMatrixVectorProduct(rdram, ctx, '
                             'GPR_U32(ctx, 3), GPR_U32(ctx, 5), GPR_U32(ctx, 2), 0x1fbdf0u); } else { '+original+' }')
            else:replacement='if (vp2sMatrixVectorReferenceEnabled()) { '+original+' }'
            return m[0][:m[0].index(body)]+body[:found.start()]+replacement+body[found.end():]
        lowered=re.sub(r'// 0x([0-9a-f]+): 0x([0-9a-f]+)[^\n]*\n(.*?)(?=\s*// 0x[0-9a-f]+:|\Z)',patch,text,flags=re.S)
        if lowered==text:continue
        include='#include "ps2_recomp_calls.h"';assert lowered.count(include)==1
        declarations=('\n'+marker+'\nbool vp2sMatrixVectorEnabled();\nbool vp2sMatrixVectorReferenceEnabled();\n'
                      'void vp2sMatrixVectorProduct(uint8_t*, R5900Context*, uint32_t, uint32_t, uint32_t, uint32_t);\n')
        path.write_text(lowered.replace(include,include+declarations));changed+=1
    print(f'Native collision point transform: USA block validated; {changed} EE files lowered')


def projection_fixtures(directory):
    # Original and rewritten EE game functions belong only to the offline
    # checker. Replace their one direct game call with a normal C++ call and
    # let bounded loops finish without scheduler checkpoints. No runtime device,
    # instruction interpreter or hardware timing is used by this harness.
    source='\n#include "ps2_runtime.h"\n#include "ps2_runtime_macros.h"\n#include "vp2s_math.h"\nnamespace vp2s::mathfixture {\n'
    for native in (False,True):
        prefix='native' if native else 'original'
        for address,name in [('216e40','Rigid'),('216cb0','ColumnRigid'),('216d60','TransposedRigid'),('214590','OffsetDot'),('2178b0','Perspective'),('2179e0','Orthographic'),('1fb240','ObjectLinear'),('1fb330','ObjectMatrix0'),('1fb530','ObjectMatrix1'),('1fb7c0','ObjectMatrix2'),('1fb6a0','ObjectPointDirection'),('1fbd60','CollisionPoint')]:
            path=directory/f'mod001_FUN_00{address}_0x{address}.cpp'
            text=path.read_text().split('// Function:',1)[1]
            text='// Function:'+text
            old=f'mod001_FUN_00{address}_0x{address}'
            assert text.count('void '+old+'(')==1
            text=text.replace(old,prefix+name)
            text=text.replace('ps2rCheckpointDue(runtime)','false')
            text=re.sub(r'#if defined\(PS2X_STRICT_RETURN_DIAGNOSTICS\).*?#else(.*?)#endif',r'\1',text,flags=re.S)
            if address in ('2178b0','2179e0'):
                text,count=re.subn(r'if \(!ps2rDispatchGuestBranch\(runtime, rdram, ctx, 0x216E40u,[^\n]+\)\) \{\s*return;\s*\}',
                                   prefix+'Rigid(rdram, ctx, runtime);',text)
                assert count==1
            if address in ('1fb240','1fb330','1fb530','1fb7c0','1fb6a0'):
                # Enter at the real continuation after the virtual callback.
                # A fixture reaching that callback is a test setup error.
                text,count=re.subn(r'if \(!ps2rDispatchGuestBranch\([^\n]+\)\) \{\s*return;\s*\}',
                                   'throw std::runtime_error("unexpected fixture callback entry");',text)
                assert count==(2 if address=='1fb330' else 1)
            if not native:
                text=re.sub(r'if \(vp2s(?:RigidInverse|ProjectionProduct|OffsetDot|LinearVector|MatrixVector|PointDirection)Enabled\(\)\) \{ [^\n]+? \} else \{ ([^\n]+?) \}',r'\1',text)
                text=re.sub(r'if \(vp2s(?:RigidInverse|ProjectionProduct|OffsetDot|LinearVector|MatrixVector|PointDirection)ReferenceEnabled\(\)\) \{ (.*?) \}',r'\1',text,flags=re.S)
                assert not re.search(r'vp2s(?:RigidInverse|ProjectionProduct|OffsetDot|TransposedRigid|LinearVector|MatrixVector|PointDirection)',text)
            assert 'ps2rDispatchGuestBranch' not in text
            source+=text+'\n'
        for index,(address,start) in enumerate([('1fb120',0x1fb1e4),('1fb240',0x1fb2fc),('1fb330',0x1fb42c),('1fb530',0x1fb624)]):
            text=(directory/f'mod001_FUN_00{address}_0x{address}.cpp').read_text()
            body=text.split(f'label_{start:x}:',1)[1].split(f'label_{start+32:x}:',1)[0]
            if not native:
                body=re.sub(r'if \(vp2sLinearVectorEnabled\(\)\) \{ [^\n]+? \} else \{ ([^\n]+?) \}',r'\1',body)
                body=re.sub(r'if \(vp2sLinearVectorReferenceEnabled\(\)\) \{ (.*?) \}',r'\1',body,flags=re.S)
                assert 'vp2sLinearVector' not in body
            source+=f'void {prefix}Linear{index}(uint8_t *rdram,R5900Context *ctx,PS2Runtime *runtime) {{\n'+body+'\n}\n'
        for index,(address,start) in enumerate([('1fb330',0x1fb3f4),('1fb530',0x1fb5ec),('1fb7c0',0x1fb87c)]):
            text=(directory/f'mod001_FUN_00{address}_0x{address}.cpp').read_text()
            body=text.split(f'label_{start:x}:',1)[1].split(f'label_{start+40:x}:',1)[0]
            if not native:
                body=re.sub(r'if \(vp2sMatrixVectorEnabled\(\)\) \{ [^\n]+? \} else \{ ([^\n]+?) \}',r'\1',body)
                body=re.sub(r'if \(vp2sMatrixVectorReferenceEnabled\(\)\) \{ (.*?) \}',r'\1',body,flags=re.S)
                assert 'vp2sMatrixVector' not in body
            source+=f'void {prefix}MatrixVector{index}(uint8_t *rdram,R5900Context *ctx,PS2Runtime *runtime) {{\n'+body+'\n}\n'
        text=(directory/'mod001_FUN_001fb6a0_0x1fb6a0.cpp').read_text()
        body=text.split('label_1fb768:',1)[1].split('label_1fb7a4:',1)[0]
        if not native:
            body=re.sub(r'if \(vp2sPointDirectionEnabled\(\)\) \{ [^\n]+? \} else \{ ([^\n]+?) \}',r'\1',body)
            body=re.sub(r'if \(vp2sPointDirectionReferenceEnabled\(\)\) \{ (.*?) \}',r'\1',body,flags=re.S)
            assert 'vp2sPointDirection' not in body
        source+=f'void {prefix}PointDirectionBlock(uint8_t *rdram,R5900Context *ctx,PS2Runtime *runtime) {{\n'+body+'\n}\n'
        text=(directory/'mod001_FUN_001fbd60_0x1fbd60.cpp').read_text()
        body=text[text.index('    // 0x1fbdcc:'):text.index('    // 0x1fbdf4:')]
        if not native:
            body=re.sub(r'if \(vp2sMatrixVectorEnabled\(\)\) \{ [^\n]+? \} else \{ ([^\n]+?) \}',r'\1',body)
            body=re.sub(r'if \(vp2sMatrixVectorReferenceEnabled\(\)\) \{ (.*?) \}',r'\1',body,flags=re.S)
            assert 'vp2sMatrixVector' not in body
        source+=f'void {prefix}CollisionPointBlock(uint8_t *rdram,R5900Context *ctx,PS2Runtime *runtime) {{\n'+body+'\n}\n'
    return source+'}\n'


def lower_normalization(directory):
    instructions={}
    for path in directory.glob('*FUN*.cpp'):
        text=path.read_text()
        if 'vcallms     0x670' not in text: continue
        for m in re.finditer(r'// 0x([0-9a-f]+): 0x([0-9a-f]+)\s+([^\n]+)',text):
            instructions[int(m[1],16)]=(int(m[2],16),m[3])
    ordered=sorted(instructions)
    metadata={}
    sites=0
    for index,pc in enumerate(ordered):
        word,operation=instructions[pc]
        if not ('vcallms' in operation and re.search(r'\b0x670\b',operation)): continue
        before=index-1
        while instructions[ordered[before]][0]==0: before-=1
        after=index+1
        while not (instructions[ordered[after]][0]>>26==0x3e and (instructions[ordered[after]][0]>>16&31)==6): after+=1
        loadpc,storepc=ordered[before],ordered[after]
        load=instructions[loadpc][0]; store=instructions[storepc][0]
        assert load>>26==0x36 and (load>>16&31)==5 and after-index<=3
        assert instructions[pc+4][0]==0x48490001
        source=(load>>21&31,load&65535)
        target=(store>>21&31,store&65535)
        # Between the input read and result store there are no EE memory writes.
        # A pointer reload must restore the same addiu definition used at input.
        for middle in ordered[index+2:after]:
            w=instructions[middle][0]
            if w==0x03e00008: continue # JR; the result store is its delay slot
            assert w>>26==9, hex(middle)
            if w>>16&31==source[0]:
                previous=[instructions[p][0] for p in ordered[:before] if instructions[p][0]>>26==9 and instructions[p][0]>>16&31==source[0]]
                assert previous and previous[-1]==w, hex(middle)
        metadata[loadpc]=('load',source,target)
        metadata[pc]=('call',source,target)
        metadata[pc+4]=('sync',source,target)
        metadata[storepc]=('store',source,target)
        sites+=1
    assert sites==16, f'Expected 16 USA quaternion normalization sites, got {sites}'
    changed=0
    for path in directory.glob('*.cpp'):
        match=re.search(r'_0x([0-9a-f]+)\.cpp$',path.name)
        if not match or not 0x215640<=int(match[1],16)<=max(metadata): continue
        text=path.read_text()
        marker='// VP2S native normalization lowering v1'
        if marker in text: continue
        def patch(m):
            pc=int(m[1],16)
            if pc not in metadata: return m[0]
            kind,source,target=metadata[pc]; body=m[2]
            if kind=='load': pattern=r'ctx->vu0_vf\[5\] = [^\n;]+;'
            elif kind=='call': pattern=r'\{[^\n]+ps2rExecuteVU0Microprogram\(runtime, rdram, ctx, 0x670\); \}'
            elif kind=='sync': pattern=r'SET_GPR_U32\(ctx, 9, static_cast<uint32_t>\(ctx->vi\[0\]\)\);'
            else: pattern=r'WRITE128\([^\n]+_mm_castps_si128\(ctx->vu0_vf\[6\]\)\);'
            found=re.search(pattern,body)
            assert found, (path.name,hex(pc),kind)
            original=found[0]
            if kind=='store':
                def addr(value): return f'ADD32(GPR_U32(ctx, {value[0]}), {value[1]})'
                replacement=('if (vp2sNormalizeEnabled()) { vp2sNormalizeRecord(rdram, ctx, '+addr(source)+', '+addr(target)+'); } '
                             'else { '+original+' }')
            elif kind=='sync':
                replacement='if (vp2sNormalizeReferenceEnabled()) { '+original+' } else { SET_GPR_U32(ctx, 9, 0u); }'
            else: replacement='if (vp2sNormalizeReferenceEnabled()) { '+original+' }'
            return m[0][:m[0].index(m[2])]+body[:found.start()]+replacement+body[found.end():]
        lowered=re.sub(r'// 0x([0-9a-f]+):[^\n]*\n(.*?)(?=\s*// 0x[0-9a-f]+:|\Z)',patch,text,flags=re.S)
        if lowered==text: continue
        include='#include "ps2_recomp_calls.h"'
        declarations=('\n'+marker+'\nbool vp2sNormalizeEnabled();\nbool vp2sNormalizeReferenceEnabled();\n'
                      'void vp2sNormalizeRecord(uint8_t*, R5900Context*, uint32_t, uint32_t);\n')
        assert lowered.count(include)==1
        path.write_text(lowered.replace(include,include+declarations))
        changed+=1
    print(f'Native quaternions: {sites} sites validated; {changed} EE files lowered')


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--elf',type=Path,required=True)
    ap.add_argument('--out',type=Path,required=True)
    ap.add_argument('--recomp',type=Path,required=True)
    args=ap.parse_args()
    data=args.elf.read_bytes()
    code=data[0x12b4a8+0x250:0x12b4a8+0x2d0]
    expected=[]
    for column in range(1,5):
        expected.extend([0x01e029bc|(column<<16),0x01e030bd|(column<<16),
                         0x01e038be|(column<<16)|(0x40000000 if column==4 else 0),
                         0x01e0400b|(column<<16)|(column<<6)])
    pairs=list(struct.iter_unpack('<II',code))
    if [a for a,b in pairs]!=[0x8000033c]*16 or [b for a,b in pairs]!=expected:
        raise ValueError('Unexpected USA matrix product; refusing native helper build')
    sys.dont_write_bytecode=True
    path=Path(__file__).resolve().parent.parent/'external/PS2Recomp/tools/vu1_lift.py'
    spec=importlib.util.spec_from_file_location('matrix_lifter',path)
    lift=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(lift)
    source,_=lift.generate(code,dict(hash=lift.code_hash(code),pairs=16,
                            function='matrix_reference',entries=[0],targets=[],unit='VU0'))
    quaternion=data[0x12b4a8+0x670:0x12b4a8+0x6b0]
    expected_q=[(0x8000033c,0x01e529aa),(0x8000033c,0x010007c3),(0x8000033c,0x0106303d),
                (0x8000033c,0x0106f8be),(0x8000033c,0x0106f98b),(0x806603be,0x000002ff),
                (0x800003bf,0x41e0299c),(0x8000033c,0x000002ff)]
    if list(struct.iter_unpack('<II',quaternion))!=expected_q:
        raise ValueError('Unexpected USA quaternion normalization algorithm')
    reference,_=lift.generate(quaternion,dict(hash=lift.code_hash(quaternion),pairs=8,
                            function='normalize_reference',entries=[0],targets=[],unit='VU0'))
    source+='\n'+reference
    conversion=data[0x12b4a8+0x720:0x12b4a8+0x7f0]
    if hashlib.sha256(conversion).hexdigest()!='5082777297b0b5b697227fc4ac15634e039e8a9c264cab5c4d0dc36da6b7096c':
        raise ValueError('Unexpected USA quaternion-to-matrix algorithm')
    reference,_=lift.generate(conversion,dict(hash=lift.code_hash(conversion),pairs=26,
                            function='quaternion_matrix_reference',entries=[0],targets=[],unit='VU0'))
    source+='\n'+reference
    args.out.mkdir(parents=True,exist_ok=True)
    path=args.out/'reference.cpp'
    lower_normalization(args.recomp/'mod001')
    lower_rigid_inverse(args.recomp/'mod001',data)
    lower_rigid_column(args.recomp/'mod001',data)
    lower_transposed_rigid(args.recomp/'mod001',data)
    lower_offset_dot(args.recomp/'mod001',data)
    lower_projection_products(args.recomp/'mod001',data)
    lower_linear_vectors(args.recomp/'mod001',data)
    lower_matrix_vectors(args.recomp/'mod001',data)
    lower_point_direction(args.recomp/'mod001',data)
    lower_collision_point(args.recomp/'mod001',data)
    source+=projection_fixtures(args.recomp/'mod001')
    source=source.encode()
    if not path.exists() or path.read_bytes()!=source: path.write_bytes(source)
    from vp2_inline_math import lower
    lower(args.recomp)
    print('Native matrix: validated USA product; original reference emitted offline')


if __name__=='__main__': main()
