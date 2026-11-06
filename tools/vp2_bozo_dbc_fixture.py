"""Strict offline native Dbc_Manager fixture; all game bytes stay in private build."""
import argparse
import json
import hashlib
import subprocess
import sys
from pathlib import Path

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('source',type=Path);p.add_argument('out',type=Path)
p.add_argument('--build',type=Path,required=True)
a=p.parse_args()
if not a.out.resolve().is_relative_to(a.build.resolve()):raise ValueError('private build directory required')
if hashlib.sha256(a.source.read_bytes()).hexdigest()!='48af0362b0229923bba1568913d1599c5b943247516a69004afba76421525dd8':
    raise ValueError('USA Dbc_Manager 3.16 source identity mismatch')
root=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(root/'external/BozoEngine/tools/native_irx'))
from relocate import relocate
from recompile import Module
a.out.mkdir(parents=True,exist_ok=True)
info=relocate(a.source,0x20000,a.out/'dbc')
m=Module(str(a.out/'dbc.json'));exports,imports=m.tables()
subprocess.run([sys.executable,'-I',str(root/'external/BozoEngine/tools/native_irx/recompile.py'),
                str(a.out/'dbc.json'),'--out',str(a.out)],check=True)
data=(a.out/'dbc.bin').read_bytes()
lines=['#pragma once','#include "bozo/native/compiled_libraries.h"',
       'inline void addVp2DbcImage(bozo::native::irx::CompiledLibraries &host) {',
       'bozo::native::irx::Image image;',f'image.name={json.dumps(info["name"])};',
       f'image.hash=0x{info["hash"]}ull;image.base={info["base"]};image.entry={info["entry"]};image.gp={info["gp"]};',
       'static const uint8_t bytes[]={']
lines+=[' '.join(f'0x{v:02x},' for v in data[i:i+32]) for i in range(0,len(data),32)]
lines+=['};','image.bytes.assign(std::begin(bytes),std::end(bytes));']
for field,entries in [('imports',imports),('exports',exports)]:
    lines+=[f'image.{field}={{']+[f'{{0x{pc:x},{json.dumps(lib)},{n}}},' for pc,(lib,n) in entries.items()]+['};']
lines+=['host.addImage(std::move(image));','}']
(a.out/'vp2_dbc_image.h').write_text('\n'.join(lines)+'\n')
