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
from recompile import Module
info=relocate(a.source,base,a.out/a.kind)
m=Module(str(a.out/(a.kind+'.json')));exports,imports=m.tables()
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
