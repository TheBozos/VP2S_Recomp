"""Compile the audited program-install producer; no VIF/DMA consumer is executed."""
import argparse
from pathlib import Path
from vp2_bozo_prepare import prepare,elf_parts
from vp2_bozo_kernel import overlay_adaptations

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('source',type=Path);p.add_argument('out',type=Path)
p.add_argument('--build',type=Path,required=True)
p.add_argument('--variant',choices=['original','adapted'],required=True)
a=p.parse_args()
if not a.out.resolve().is_relative_to(a.build.resolve()):raise ValueError('private build directory required')
data=a.source.read_bytes()
patches=overlay_adaptations('mod001',data)
fixture,_=prepare(data,[dict(Name='program_install',Start='0x29e2e0',End='0x29e310'),
                       dict(Name='math_install',Start='0x29e150',End='0x29e180'),
                       dict(Name='camera_publication',Start='0x208900',End='0x208aac'),
                       dict(Name='frustum_publication',Start='0x207cc0',End='0x207d70'),
                       dict(Name='visibility_inputs',Start='0x2205c0',End='0x220d14'),
                       dict(Name='render_job',Start='0x29e420',End='0x29e4a4'),
                       dict(Name='scene_wait',Start='0x221030',End='0x2210b8')],{})
a.out.mkdir(parents=True,exist_ok=True)
(a.out/'fixture.elf').write_bytes(fixture)
(a.out/'externals.txt').write_text('stub 0x201f20\nstub 0x170100\nstub 0x2138f0\nstub 0x215870\n'+('native 0x29e160\nnative 0x208a94\n' if a.variant=='adapted' else '')+''.join(f'native 0x{pc:x}\n' for pc in [*range(0x207cc8,0x207d68,4),0x207d6c]))
(a.out/'patches.txt').write_text(''.join(line+'\n' for line in patches.splitlines() if line.split()[0] in ('0029e2fc','0029e160','0029e16c','00208a90')))
_,segments,_=elf_parts(data);lo,hi=0x2205c0,0x220d14;s=next(s for s in segments if s[0]==1 and s[2]<=lo<hi<=s[2]+s[4])
with (a.out/'externals.txt').open('a') as out:
    out.write('native 0x2205c0\nnative 0x220ce0\nstub 0x20f440\nstub 0x2082f0\nstub 0x208cd0\nstub 0x100af0\nstub 0x100b30\nstub 0x23e000\n')
    out.write('native 0x29e43c\nnative 0x29e444\nnative 0x22106c\nstub 0x21fab0\nstub 0x10d0f0\nstub 0x101080\nstub 0x10e820\nstub 0x11f4b0\nstub 0x10e290\n')
    for pc in range(lo,hi,4):
        word=int.from_bytes(data[s[1]+pc-s[2]:s[1]+pc-s[2]+4],'little')
        if word>>26 in (0x12,0x36,0x3e):out.write(f'native 0x{pc:x}\n')
