"""Validate USA matrix helper and emit its original algorithm for offline checking.

The native game uses a typed C++ matrix operation, never these reference files.
"""
import argparse
import importlib.util
import struct
import sys
import re
from pathlib import Path


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
    args.out.mkdir(parents=True,exist_ok=True)
    path=args.out/'reference.cpp'
    source=source.encode()
    if not path.exists() or path.read_bytes()!=source: path.write_bytes(source)
    lower_normalization(args.recomp/'mod001')
    print('Native matrix: validated USA product; original reference emitted offline')


if __name__=='__main__': main()
