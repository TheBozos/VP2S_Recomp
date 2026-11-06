"""Compile only the private USA controller record reference; never run its hardware worker."""
import argparse, hashlib, sys, subprocess
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
info=relocate(a.source,0x26000,a.out/'ds2')
if info['hash']!='c477e2507b82e6d5':raise ValueError('USA ds2o_d source identity mismatch')
subprocess.run([sys.executable,'-I',str(root/'external/BozoEngine/tools/native_irx/recompile.py'),
    str(a.out/'ds2.json'),'--out',str(a.out)],check=True)
source=a.out/'ds2o_d_iop.cpp'
with source.open('a') as f:
    f.write('\nvoid vp2ControllerReference(bozo::native::irx::Env &e){iop_c477e250_02722c(e);}\n')
