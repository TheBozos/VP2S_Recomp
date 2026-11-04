"""Isolated, non-activating USA new-game -> Attacks -> exit twice probe.

Run from Windows: python tools/attacks_menu_probe.py --tag after --exe <exe>
Screenshots and logs live in build_usa/attacks_probe/<tag>. Inspect the captures;
absence of a missing-target log alone does not prove the menu was reached.
"""
import argparse
import os
from pathlib import Path
import shutil
import subprocess
import time

root = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--tag', required=True)
parser.add_argument('--exe', type=Path, default=root / 'build_usa/port/vp2s.exe')
args = parser.parse_args()
if not args.tag or any(c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-' for c in args.tag):
    parser.error('tag must contain only letters, numbers, underscore or hyphen')
out = root / 'build_usa/attacks_probe' / args.tag
out.mkdir(parents=True, exist_ok=False)
iso = out / 'iso'
source = root / 'build_usa/extract/iso'
# Game files are read-only hard links; cards are independent copies. Nothing in
# the user's ISO/card tree is removed, restored or written by this probe.
for directory, folders, files in os.walk(source):
    rel = Path(directory).relative_to(source)
    if rel == Path('.'):
        for card in ('mc0', 'mc1'):
            if card in folders:
                folders.remove(card)
                shutil.copytree(source / card, iso / card)
    target = iso / rel
    target.mkdir(parents=True, exist_ok=True)
    for name in files:
        os.link(Path(directory) / name, target / name)
exe = out / 'vp2s_probe.exe'
shutil.copy2(args.exe, exe)
shots = out / 'shots'
shots.mkdir()
(out / 'pad.txt').write_text('')
env = os.environ.copy()
for key in list(env):
    if key.startswith(('PS2X_PAD_', 'PS2X_NATIVE_CAPTURE', 'VP2S_DIAG')):
        del env[key]
script = '25:SQUARE,30:START,36:START,42:CROSS,'
script += ','.join(f'{t}:START,{t+1}:CROSS' for t in range(80, 108, 4))
script += ',135:SELECT/50,141:DOWN/50,143:DOWN/50,145:DOWN/50,148:CROSS/50,154:CROSS/50'
script += ',164:CIRCLE/50,174:CROSS/50,178:CROSS/50,184:CIRCLE/50,194:CIRCLE/50,198:CIRCLE/50,206:RIGHT/500'
env.update(VP2S_CLEAN='1', PS2X_PAD_NO_HOST='1', PS2X_WINDOW_NO_FOCUS='1',
           PS2X_AUDIO_MUTE='1', PS2X_MPEG_SKIP='30', PS2X_PAD_SCRIPT=script,
           PS2X_PAD_INPUT=str(out / 'pad.txt'), PS2X_NATIVE_CAPTURE_DIR=str(shots),
           PS2X_NATIVE_CAPTURE_INTERVAL='1')
launcher_output = out / 'launcher.txt'
with launcher_output.open('w') as launch_log:
    subprocess.run(['powershell', '-NoProfile', '-ExecutionPolicy', 'Bypass', '-File',
                         str(root / 'build/agent_tools/launch_noactivate.ps1'),
                         '-Exe', str(exe), '-Arguments', str(iso / 'SLUS_214.52'),
                         '-Out', str(out / 'run.out'), '-Err', str(out / 'run.err'),
                         '-Dir', str(root / 'build_usa/port'), '-Show', '7'],
                        env=env, stdout=launch_log, check=True)
pid = int(launcher_output.read_text().strip())
(out / 'pid.txt').write_text(str(pid))
print(f'Isolated probe PID {pid}: {out}', flush=True)
try:
    time.sleep(3)
    # Minimized native presentation can stall. Restore outside every monitor
    # without activation; this helper verifies both bounds and foreground.
    subprocess.run(['powershell', '-NoProfile', '-ExecutionPolicy', 'Bypass', '-File',
                    str(root / 'tools/place_probe_offscreen.ps1'), '-GamePid', str(pid)], check=True)
    time.sleep(215)
finally:
    subprocess.run(['powershell', '-NoProfile', '-Command',
                    f'Stop-Process -Id {pid} -Force -ErrorAction SilentlyContinue'], check=False)
log = (out / 'run.err').read_text(errors='replace')
missing = '[guest-branch:missing-target]' in log
print('missing-target:', missing, flush=True)
raise SystemExit(1 if missing else 0)
