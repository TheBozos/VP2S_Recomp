"""Admit the private USA libsd image to the native mixer, with strict offline arithmetic."""
import argparse,hashlib,json,subprocess,sys
from pathlib import Path
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('source',type=Path);p.add_argument('out',type=Path);p.add_argument('--build',required=True,type=Path)
a=p.parse_args()
if not a.out.resolve().is_relative_to(a.build.resolve()):raise ValueError('private build directory required')
if hashlib.sha256(a.source.read_bytes()).hexdigest()!='d26a8dcd121ff4ded9c819b72864ea7f4e723932cc2eab1b4f0f1c93faad43ef':
    raise ValueError('USA Sound_Device_Library identity mismatch')
root=Path(__file__).resolve().parents[1];a.out.mkdir(parents=True,exist_ok=True)
sys.path.insert(0,str(root/'external/BozoEngine/tools/native_irx'))
from relocate import relocate
from recompile import Module
info=relocate(a.source,0x30000,a.out/'libsd')
m=Module(str(a.out/'libsd.json'));exports,imports=m.tables()
subprocess.run([sys.executable,'-I',str(root/'external/BozoEngine/tools/native_irx/recompile.py'),
    str(a.out/'libsd.json'),'--out',str(a.out)],check=True)
data=(a.out/'libsd.bin').read_bytes()
lines=['#pragma once','#include "bozo/native/compiled_libraries.h"',
       'inline void addVp2SoundImage(bozo::native::irx::CompiledLibraries &host) {',
       'bozo::native::irx::Image image;',f'image.name={json.dumps(info["name"])};',
       f'image.hash=0x{info["hash"]}ull;image.base={info["base"]};image.entry={info["entry"]};image.gp={info["gp"]};',
       'static const uint8_t bytes[]={']
lines+=[' '.join(f'0x{v:02x},' for v in data[i:i+32]) for i in range(0,len(data),32)]
lines+=['};','image.bytes.assign(std::begin(bytes),std::end(bytes));']
for field,entries in [('imports',imports),('exports',exports)]:
    lines+=[f'image.{field}={{']+[f'{{0x{pc:x},{json.dumps(lib)},{n}}},' for pc,(lib,n) in entries.items()]+['};']
lines+=['image.nativeLibsd=true;','host.addImage(std::move(image));','}']
(a.out/'vp2_sound_image.h').write_text('\n'.join(lines)+'\n')
