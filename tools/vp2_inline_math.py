"""Lower validated USA quaternion record operations, including carved EE entries."""
from pathlib import Path
import re

INSTRUCTION=re.compile(r'// 0x([0-9a-f]+): 0x([0-9a-f]+)\s+([^\n]+)')
SPECIAL={0x48426c:(0x50,0x4842d0),0x485724:(0xb0,0x485788),0x48c9c4:(0x90,0x48ca24)}

def immediate(word):
    v=word&65535
    return v-65536 if v&32768 else v

def address(record):
    reg,offset=record
    return f'ADD32(GPR_U32(ctx, {reg}), {offset & 0xffffffff}u)'

def lower(directory):
    total=changed=0
    for module in ('mod001','mod003','mod028'):
        folder=directory/module
        instructions={}; owners={}
        for path in folder.glob('*FUN*.cpp'):
            text=path.read_text()
            if 'vcallms     0x720' not in text: continue
            function=int(path.stem.rsplit('_0x',1)[1],16)
            for m in INSTRUCTION.finditer(text):
                pc=int(m[1],16)
                instructions[pc]=(int(m[2],16),m[3]); owners[pc]=function
        metadata={}; ranges={}
        for pc,(word,op) in sorted(instructions.items()):
            if op!='vcallms     0x720' or pc==0x21673c: continue
            assert word==0x4a003938,hex(pc)
            load=instructions[pc-8][0]
            assert load>>26==0x36 and (load>>16&31)==5 and instructions[pc-4][0]==0
            source=(load>>21&31,immediate(load))
            metadata[pc-8]=('load',None)
            if pc in SPECIAL:
                offset,begin=SPECIAL[pc]
                destination=(29,offset);columns=4
                # Native rotation lives in the routine's existing final matrix
                # stack slot across the object callback, never in VU shadow state.
                end=begin+23*4
                for column in range(4):
                    w=instructions[begin+column*4][0]
                    assert w>>26==0x36 and (w>>16&31)==column+1 and immediate(w)==column*16
                input_record=(instructions[begin][0]>>21&31,0)
                for k in range(begin,begin+20*4,4):
                    metadata[k]=('reference',None)
                # Validate the exact four-column multiply/add sequence, not an
                # arbitrary run of macro instructions at matching addresses.
                for column in range(4):
                    expected=[0x4be041bc|((column+1)<<16),0x4be048bd|((column+1)<<16),
                              0x4be050be|((column+1)<<16),
                              0x4be0580b|((column+1)<<16)|((column+1)<<6)]
                    actual=[instructions[begin+16+column*16+j*4][0] for j in range(4)]
                    assert actual==expected,hex(pc)
                for column in range(4):
                    k=begin+80+column*4;w=instructions[k][0]
                    assert w>>26==62 and (w>>16&31)==column+1 and immediate(w)==column*16
                    if column==0:
                        metadata[k]=('compose',(input_record,(w>>21&31,immediate(w))))
                    else: metadata[k]=('reference',None)
            else:
                # Symbolically resolve the destination to registers as they are
                # at the call. Only translation-column writes may intervene.
                expressions={r:(r,0) for r in range(32)}
                stores=[];k=pc+4
                while True:
                    w,operation=instructions[k];opcode=w>>26
                    if opcode==62 and (w>>16&31)==8:break
                    if opcode==9:
                        base,off=expressions[w>>21&31]
                        expressions[w>>16&31]=(base,off+immediate(w))
                    elif opcode==15:expressions[w>>16&31]=None
                    elif w==0x48490001:
                        metadata[k]=('sync',None);expressions[9]=None
                    elif opcode==57 or (opcode==62 and (w>>16&31)==0):
                        base,off=expressions[w>>21&31]
                        stores.append((base,off+immediate(w)))
                        if opcode==62:metadata[k]=('constant',None)
                    else:assert w==0 or (opcode==17 and w>>21&31==4),hex(k)
                    k+=4
                    assert k-pc<=80,hex(pc)
                w=instructions[k][0];base,off=expressions[w>>21&31]
                destination=(base,off+immediate(w))
                # Earlier scalar stores initialize only the fourth column.
                assert all(r==base and destination[1]+48<=o<=destination[1]+60 for r,o in stores),hex(pc)
                columns=4 if module=='mod001' else 3
                for column in range(columns):
                    w=instructions[k+column*4][0]
                    assert w>>26==62 and (w>>16&31)==8+column
                    assert (w>>21&31)==(instructions[k][0]>>21&31) and immediate(w)==column*16
                    metadata[k+column*4]=('reference',None)
                end=k+(columns-1)*4
            metadata[pc]=('convert',(source,destination,columns,pc))
            # All synchronization and constant-column stores before the result
            # consumer are ordinary EE effects in the native path.
            for k in range(pc+4,min(end,pc+80),4):
                w=instructions.get(k,(0,''))[0]
                if w==0x48490001:metadata[k]=('sync',None)
                if w>>26==62 and (w>>16&31)==0:metadata[k]=('constant',None)
            ranges[owners[pc]]=max(end,ranges.get(owners[pc],0))
            total+=1
        for path in folder.glob('*.cpp'):
            match=re.search(r'_0x([0-9a-f]+)$',path.stem)
            if not match:continue
            entry=int(match[1],16)
            if not any(start<=entry<=end for start,end in ranges.items()):continue
            text=path.read_text();marker='// VP2S native inline quaternion matrices v1'
            if marker in text:continue
            def patch(m):
                pc=int(m[1],16)
                if pc not in metadata:return m[0]
                kind,payload=metadata[pc];body=m[2]
                if kind=='load' or (kind=='reference' and instructions[pc][0]>>26==54):
                    pattern=r'ctx->vu0_vf\[\d+\] = [^\n;]+;'
                elif kind=='convert':pattern=r'\{[^\n]+ps2rExecuteVU0Microprogram\(runtime, rdram, ctx, 0x720\); \}'
                elif kind=='sync':pattern=r'SET_GPR_U32\(ctx, 9, static_cast<uint32_t>\(ctx->vi\[0\]\)\);'
                elif kind in ('compose','constant') or instructions[pc][0]>>26==62:
                    pattern=r'WRITE128\([^\n]+_mm_castps_si128\(ctx->vu0_vf\[\d+\]\)\);'
                else:pattern=r'\{ __m128.*? \}'
                found=re.search(pattern,body,re.S);assert found,(path.name,hex(pc),kind)
                original=found[0]
                if kind=='convert':
                    source,destination,columns,site=payload
                    replacement=('if (vp2sInlineQuaternionReferenceEnabled()) { '+original+' } '
                                 'if (vp2sInlineQuaternionEnabled()) { vp2sInlineQuaternionMatrix(rdram, ctx, '+
                                 address(source)+', '+address(destination)+f', {columns}u, 0x{site:x}u); }}')
                elif kind=='compose':
                    source,destination=payload
                    replacement=('if (vp2sInlineQuaternionEnabled()) { vp2sInlineQuaternionCompose(rdram, ctx, '+
                                 address(source)+', '+address(destination)+'); } else { '+original+' }')
                elif kind=='sync':replacement='if (vp2sInlineQuaternionReferenceEnabled()) { '+original+' } else { SET_GPR_U32(ctx, 9, 0u); }'
                elif kind=='constant':
                    replacement=original.replace('_mm_castps_si128(ctx->vu0_vf[0])','_mm_set_epi32(0x3f800000, 0, 0, 0)')
                else:replacement='if (vp2sInlineQuaternionReferenceEnabled()) { '+original+' }'
                return m[0][:m[0].index(body)]+body[:found.start()]+replacement+body[found.end():]
            lowered=re.sub(r'// 0x([0-9a-f]+):[^\n]*\n(.*?)(?=\s*// 0x[0-9a-f]+:|\Z)',patch,text,flags=re.S)
            if lowered==text:continue
            include='#include "ps2_recomp_calls.h"'
            declarations=('\n'+marker+'\nbool vp2sInlineQuaternionEnabled();\nbool vp2sInlineQuaternionReferenceEnabled();\n'
                          'void vp2sInlineQuaternionMatrix(uint8_t*, R5900Context*, uint32_t, uint32_t, unsigned, uint32_t);\n'
                          'void vp2sInlineQuaternionCompose(uint8_t*, R5900Context*, uint32_t, uint32_t);\n')
            assert lowered.count(include)==1
            path.write_text(lowered.replace(include,include+declarations));changed+=1
    assert total==56,total
    print(f'Native inline quaternion matrices: {total} sites validated; {changed} EE files lowered')
