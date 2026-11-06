"""Compile only the private USA controller record reference; never run its hardware worker."""
import argparse, hashlib, sys, subprocess, json
from pathlib import Path
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('source',type=Path);p.add_argument('out',type=Path);p.add_argument('--build',required=True,type=Path)
a=p.parse_args()
if not a.out.resolve().is_relative_to(a.build.resolve()):raise ValueError('private build directory required')
root=Path(__file__).resolve().parents[1]
if hashlib.sha256(a.source.read_bytes()).hexdigest()!='cd9b885dc50535defbca97cadaee27d28cb71733347f8165a208634aee062e20':
    raise ValueError('USA ds2o_d source identity mismatch')
a.out.mkdir(parents=True,exist_ok=True)
sys.path.insert(0,str(root/'external/BozoEngine/tools/native_irx'))
from relocate import relocate
from recompile import Module
info=relocate(a.source,0x26000,a.out/'ds2')
if info['hash']!='c477e2507b82e6d5':raise ValueError('USA ds2o_d source identity mismatch')
subprocess.run([sys.executable,'-I',str(root/'external/BozoEngine/tools/native_irx/recompile.py'),
    str(a.out/'ds2.json'),'--out',str(a.out)],check=True)
source=a.out/'ds2o_d_iop.cpp'
with source.open('a') as f:
    f.write('\nvoid vp2ControllerReference(bozo::native::irx::Env &e){iop_c477e250_02722c(e);}\n')
m=Module(str(a.out/'ds2.json'));exports,imports=m.tables()
data=(a.out/'ds2.bin').read_bytes()
lines=['#pragma once','#include "bozo/native/compiled_libraries.h"',
       'inline void addVp2ControllerImage(bozo::native::irx::CompiledLibraries &host) {',
       'bozo::native::irx::Image image;',f'image.name={json.dumps(info["name"])};',
       f'image.hash=0x{info["hash"]}ull;image.base={info["base"]};image.entry={info["entry"]};image.gp={info["gp"]};',
       'static const uint8_t bytes[]={']
lines+=[' '.join(f'0x{v:02x},' for v in data[i:i+32]) for i in range(0,len(data),32)]
lines+=['};','image.bytes.assign(std::begin(bytes),std::end(bytes));']
for field,entries in [('imports',imports),('exports',exports)]:
    lines+=[f'image.{field}={{']+[f'{{0x{pc:x},{json.dumps(lib)},{n}}},' for pc,(lib,n) in entries.items()]+['};']
lines+=['host.addImage(std::move(image));','}']
(a.out/'vp2_controller_image.h').write_text('\n'.join(lines)+'\n')
