"""Native Windows BozoEngine bring-up pipeline, independent of PS2Recomp.

This is an initial migration runner, not the playable VP2S launcher. Unported
services/device/VU boundaries fail explicitly. Generated game data stays local.
"""
import argparse
import json
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path
from vp2_bozo_prepare import elf_parts
from vp2_bozo_kernel import bindings as kernel_bindings, game_adaptations, frame_adaptations, native_game_sites,bootstrap_adaptations,overlay_adaptations,overlay_native_sites,display_adaptations,startup_clear_adaptations,composition_adaptations,debug_font_adaptations,reject_performance_counter_reads

ROOT = Path(__file__).resolve().parent.parent


def run(command, log, env):
    log.parent.mkdir(parents=True, exist_ok=True)
    with log.open('w', encoding='utf-8') as out:
        result = subprocess.run([str(v) for v in command], cwd=ROOT, env=env,
                                stdout=out, stderr=subprocess.STDOUT)
    if result.returncode:
        print(log.read_text(errors='replace')[-5000:])
        raise SystemExit(f'failed ({result.returncode}): {log}')
    return log.read_text(errors='replace')


def generate(args, build, data, engine_build, env):
    run([sys.executable, ROOT/'tools/vp2_bozo_meshes.py','--source',build/'vp2_meshes',
         '--out',data/'meshes'],data/'meshes.log',env)
    run([sys.executable, ROOT/'tools/vp2_bozo_prepare.py', '--build', build,
         '--out', data/'images', '--inputs', args.inputs], data/'prepare.log', env)
    run([sys.executable,ROOT/'tools/vp2_music.py','--elf',data/'images/main.elf',
         '--out',build/'native_music'],data/'native-music.log',env)
    manifest = json.loads((data/'images/manifest.json').read_text())
    if 'main' not in manifest:
        raise SystemExit('the bring-up runner requires main in --inputs')
    tool = engine_build/'engine/recompiler/bozo-recomp.exe'
    externals = engine_build/'engine/tools/externals/bozo-externals.exe'
    if os.name != 'nt':
        tool = tool.with_suffix(''); externals = externals.with_suffix('')
    sources, includes, registrations = [], [], []
    for name, info in manifest.items():
        image = data/'images'/f'{name}.elf'
        prefix = f'vp2_{name}'
        target = data/'generated'/name
        ext = data/'images'/f'{name}.externals'
        run([externals, image, ext], data/'logs'/f'{name}_externals.log', env)
        # Fail closed at every remaining COP2/VU register operation. The engine
        # supports macro-mode compatibility code; this port does not accept it
        # as a migration fallback. Later native algorithms replace these sites.
        raw = image.read_bytes()
        reject_performance_counter_reads(raw)
        adaptations=[]
        if name == 'main':
            with ext.open('a') as stream:
                stream.write(kernel_bindings(raw))
                stream.write(native_game_sites(raw))
                stream.write('stub 0x105b80\nstub 0x1060a0\n')
            patches=data/'images'/f'{name}.patches'
            patches.write_text(game_adaptations(raw)+frame_adaptations(raw)+bootstrap_adaptations(raw)+display_adaptations(raw)+startup_clear_adaptations(raw)+composition_adaptations(raw)+debug_font_adaptations(raw))
            adaptations=['--patches',patches]
        else:
            with ext.open('a') as stream:stream.write(overlay_native_sites(name,raw))
            native_patches=overlay_adaptations(name,raw)
            if native_patches:
                patches=data/'images'/f'{name}.native.patches';patches.write_text(native_patches)
                adaptations=['--patches',patches]
        _, _, sections = elf_parts(raw)
        vu_sites = []
        for section in sections:
            if section[2] & 6 != 6:
                continue
            for offset in range(0, section[5], 4):
                word = int.from_bytes(raw[section[4]+offset:section[4]+offset+4], 'little')
                if word >> 26 in (0x12, 0x36, 0x3e):
                    vu_sites.append(section[3]+offset)
        with ext.open('a') as stream:
            stream.write(''.join(f'native 0x{pc:x}\n' for pc in vu_sites))
        info['guarded_vu_sites'] = len(vu_sites)
        run([tool, image, '--census'], data/'logs'/f'{name}_census.log', env)
        count = 16 if name in ('main', 'mod001', 'mod003') else 4
        text = run([tool, image, target, '--prefix', prefix, '--symbol-prefix', prefix+'_',
                    '--files', count, '--externals', ext, '--entries', data/'images'/f'{name}.entries', '--no-comments']+adaptations,
                   data/'logs'/f'{name}_generate.log', env)
        unknown = re.search(r'untranslated instructions: (\d+)', text)
        if unknown is None:
            raise SystemExit(f'{name}: missing generation summary')
        info['untranslated'] = int(unknown[1])
        info['chunks'] = count
        sources += [target/f'{prefix}_{i:03d}.cpp' for i in range(count)] + [target/f'{prefix}_table.cpp']
        includes.append(f'#include "{name}/{prefix}_functions.h"')
        if name != 'main':
            registrations += [f'    add("{name}",0x{info["load"]:x}u,0x{info["end"]:x}u,',
                              f'        {prefix}_function_table,{prefix}_function_count,{prefix}_resume_table,{prefix}_resume_count,{prefix}_external_table,{prefix}_external_count);']
        print(f'{name}: generated {count} chunks; {info["untranslated"]} explicit untranslated boundaries')
    header = ['// Generated locally from image metadata. No game bytes embedded.', '#pragma once',
              '#include "bozo/runtime.h"', '#include <fstream>', '#include <iterator>',
              '#include <stdexcept>', '#include <string>', '#include <vector>', '#include <functional>'] + includes
    header += ['inline void vp2sBozoRegisterImages(Runtime &rt,const char *root,const std::function<void(uint32_t)> &externalOwner) {',
               '    auto add=[&](const char *name,uint32_t lo,uint32_t hi,const BozoFunctionEntry *fn,size_t nf,const BozoResumeEntry *resume,size_t nr,const BozoExternalEntry *externals,size_t ne) {',
               '        std::ifstream in(std::string(root)+"/"+name+".identity",std::ios::binary);',
               '        std::vector<uint8_t> identity{std::istreambuf_iterator<char>(in),{}};',
               '        if(identity.size()!=64 || lo>=hi || hi>0x2000000u)throw std::runtime_error("invalid native overlay identity/range");',
               '        rt.functions.addModule(lo,hi,rt.memory.ram()+lo,std::move(identity),fn,nf,resume,nr);',
               '        for(size_t i=0;i<ne;++i){if(externals[i].kind!=1)throw std::runtime_error("unported overlay override kind");externalOwner(externals[i].address);}',
               '    };'] + registrations + ['}', '']
    (data/'generated/vp2_bozo_images.h').write_text('\n'.join(header))
    (data/'sources.cmake').write_text('set(VP2S_BOZO_SOURCES\n' + ''.join(f'  "{p.as_posix()}"\n' for p in sources) + ')\n')
    (data/'generation.json').write_text(json.dumps(manifest, indent=2)+'\n')


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--build', type=Path, default=ROOT/'build_usa')
    ap.add_argument('--inputs', default='main,mod001,mod002')
    ap.add_argument('--stage', choices=['tools', 'generate', 'build', 'run', 'all'], default='build')
    ap.add_argument('--jobs', type=int, default=4)
    ap.add_argument('--ffmpeg', default=os.environ.get('BOZO_FFMPEG_ROOT', 'D:/tmb_kit/kit/ffmpeg'))
    ap.add_argument('--iso', type=Path, default=ROOT/'Valkyrie Profile 2 - Silmeria (USA).iso')
    args = ap.parse_args()
    if args.jobs < 1 or args.jobs > 16:
        raise SystemExit('jobs must be in 1..16')
    build = args.build.resolve(); data = build/'bozo'; engine_build = build/'bozo_port'
    env = dict(os.environ)
    native_bin = Path('C:/msys64/ucrt64/bin')
    if os.name == 'nt':
        env['PATH'] = str(native_bin) + os.pathsep + env['PATH']
    cmake = shutil.which('cmake', path=env['PATH'])
    if not cmake:
        raise SystemExit('native CMake is required')
    configure = [cmake, '-S', ROOT/'bozo', '-B', engine_build, '-G', 'Ninja',
                 '-DCMAKE_BUILD_TYPE=Release', f'-DVP2S_BOZO_DATA={data.as_posix()}',
                 f'-DVP2S_BOZO_TEST_ISO={args.iso.resolve().as_posix() if args.iso.is_file() else ""}',
                 f'-DBOZO_FFMPEG_ROOT={args.ffmpeg}']
    if os.name == 'nt':
        configure += [f'-DCMAKE_CXX_COMPILER={(native_bin/"g++.exe").as_posix()}']
    run(configure+['-DVP2S_BOZO_CODE=OFF'], data/'configure-tools.log', env)
    run([cmake, '--build', engine_build, '--target', 'bozo-recomp', 'bozo-externals', '-j', args.jobs],
        data/'build-tools.log', env)
    if args.stage == 'tools':
        return
    generate(args, build, data, engine_build, env)
    run([sys.executable,ROOT/'tools/vp2_bozo_texture_manifest.py',build/'vp2_texture_index.json',
         build/'vp2_textures',data/'textures.manifest','--module',data/'images/mod001.elf'],data/'texture-manifest.log',env)
    run([sys.executable,ROOT/'tools/vp2_glyphs.py','--assets',build/'extract/assets',
         '--textures',build/'vp2_textures','--texture-manifest',data/'textures.manifest','--out',data/'glyphs'],data/'glyph-conversion.log',env)
    if args.stage == 'generate':
        return
    run(configure+['-DVP2S_BOZO_CODE=ON'], data/'configure-port.log', env)
    run([cmake, '--build', engine_build, '-j', args.jobs],
        data/'build-port.log', env)
    run([Path(cmake).with_name('ctest.exe' if os.name == 'nt' else 'ctest'),
         '--test-dir', engine_build, '--output-on-failure'], data/'tests.log', env)
    print(f'Built {engine_build}/vp2s-bozo-headless')
    if args.stage in ('run', 'all'):
        exe = engine_build/('vp2s-bozo-headless.exe' if os.name == 'nt' else 'vp2s-bozo-headless')
        # Headless only: no host input or window. Omitted card-dir means fresh
        # in-memory cards; the user's filesystem saves are never opened.
        run([exe, '--iso', args.iso, '--frames', '1'], data/'bringup.log', env)


if __name__ == '__main__':
    main()
