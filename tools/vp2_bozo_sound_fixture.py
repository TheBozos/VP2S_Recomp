"""Admit the private USA libsd image to the native mixer, with strict offline arithmetic."""
import argparse,hashlib,json,subprocess,sys
from pathlib import Path
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('source',type=Path);p.add_argument('out',type=Path);p.add_argument('--build',required=True,type=Path)
p.add_argument('--kind',choices=['libsd','csm','csd','csi'],default='libsd')
a=p.parse_args()
if not a.out.resolve().is_relative_to(a.build.resolve()):raise ValueError('private build directory required')
identities={'libsd':('d26a8dcd121ff4ded9c819b72864ea7f4e723932cc2eab1b4f0f1c93faad43ef',0x30000,'Sound'),
            'csm':('09d4daf88a61d2756a8f93cd2e1be4930f55d3ba6d59d92d52aa1a17df4835a7',0x40000,'Csm'),
            'csd':('3196c1f9ed3597d0cb0d46ce4979ec0ec4630476bcee56f0d74f42a0aa758f00',0x50000,'Csd'),
            'csi':('da508580b89f62ffee7037b26e21086e1944cf8aa48f21cb80b796580503c2e5',0x68000,'Csi')}
digest,base,label=identities[a.kind]
if hashlib.sha256(a.source.read_bytes()).hexdigest()!=digest:
    raise ValueError(f'USA {a.kind} identity mismatch')
root=Path(__file__).resolve().parents[1];a.out.mkdir(parents=True,exist_ok=True)
sys.path.insert(0,str(root/'external/BozoEngine/tools/native_irx'))
from relocate import relocate
from recompile import Module,find_functions,emit_module,emit_function,name_of
info=relocate(a.source,base,a.out/a.kind)
m=Module(str(a.out/(a.kind+'.json')));exports,imports=m.tables()
if a.kind=='csm':
    # Native audio starts on its own scheduler. Only this startup alignment wait
    # for a physical display edge is retired; frame listeners consume real frames.
    if m.word(0x449f0)!=0x0c0115f1 or m.word(0x449f4)!=0:
        raise ValueError('USA Csm display-alignment call mismatch')
    original=find_functions(m,exports,imports,[])
    body=emit_function(m,original[0x44978],original,True)
    reference=['#include "bozo/native/compiled_irx.h"','#include <stdexcept>',
               'namespace vp2_csm_reference {','using namespace bozo::native::irx;']+body+['}',
               'void vp2CsmTimerReference(bozo::native::irx::Env &e){',
               f'vp2_csm_reference::{name_of(m,0x44978)}(e);','}']
    (a.out/'csm_timer_reference.cpp').write_text('\n'.join(reference)+'\n')
    original_word=m.word
    m.word=lambda address:0 if address==0x449f0 else original_word(address)
    functions=find_functions(m,exports,imports,[])
    if any(fn.unsafe for fn in functions.values()):raise ValueError('unsafe adapted Csm function')
    (a.out/'csm__iop.cpp').write_text(emit_module(m,functions,True)+
        '\nvoid vp2CsmTimerNative(bozo::native::irx::Env &e){'+name_of(m,0x44978)+'(e);}\n')
else:
    subprocess.run([sys.executable,'-I',str(root/'external/BozoEngine/tools/native_irx/recompile.py'),
        str(a.out/(a.kind+'.json')),'--out',str(a.out)],check=True)
data=(a.out/(a.kind+'.bin')).read_bytes()
lines=['#pragma once','#include "bozo/native/compiled_libraries.h"',
       f'inline void addVp2{label}Image(bozo::native::irx::CompiledLibraries &host) {{',
       'bozo::native::irx::Image image;',f'image.name={json.dumps(info["name"])};',
       f'image.hash=0x{info["hash"]}ull;image.base={info["base"]};image.entry={info["entry"]};image.gp={info["gp"]};',
       'static const uint8_t bytes[]={']
lines+=[' '.join(f'0x{v:02x},' for v in data[i:i+32]) for i in range(0,len(data),32)]
lines+=['};','image.bytes.assign(std::begin(bytes),std::end(bytes));']
for field,entries in [('imports',imports),('exports',exports)]:
    lines+=[f'image.{field}={{']+[f'{{0x{pc:x},{json.dumps(lib)},{n}}},' for pc,(lib,n) in entries.items()]+['};']
lines+=[f'image.nativeLibsd={str(a.kind=="libsd").lower()};','host.addImage(std::move(image));','}']
(a.out/f'vp2_{label.lower()}_image.h').write_text('\n'.join(lines)+'\n')
