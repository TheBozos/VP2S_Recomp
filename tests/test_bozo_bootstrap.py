"""Bounded headless integration through native timer callbacks to SDK CD worker.

No window/input options or card directory are passed. The runner uses fresh
in-memory cards. The known mixed SDK CD worker still fails the no-frame guard;
this test qualifies that boundary only, not a playable route or visual parity.
"""
import subprocess
import sys
import os

result=subprocess.run([sys.argv[1],'--iso',sys.argv[2],'--frames','1'],
                      capture_output=True,text=True,timeout=30,
                      env={**os.environ,'VP2S_BOZO_TRACE_CALLS':'1'})
output=result.stdout+result.stderr
print(output,end='')
if result.returncode!=1:
    raise SystemExit(f'expected explicit unported boundary, got {result.returncode}')
if '[vp2s:bozo] native syscall services ready; retired SDK firmware discovery' not in output:
    raise SystemExit('startup did not reach the native installer')
if '[vp2s:bozo] retired SDK alarm firmware installer; runtime alarm APIs remain unsupported' not in output:
    raise SystemExit('startup did not retire the alarm firmware installer')
if '[vp2s:bozo] native timer records initialized' not in output:
    raise SystemExit('startup did not reach native timer record initialization')
if '[vp2s:bozo] native SDK timer started' not in output:
    raise SystemExit('startup did not start the native SDK clock')
if '[vp2s:bozo] retired SDK event firmware installer' not in output:
    raise SystemExit('startup did not retire the audited event firmware installer')
for required in ['[vp2s:call] pc=142150 ra=0',
                 'pc=137220 a0=ffffffff result=-408',
                 'no game frame ended for 120 fields']:
    if required not in output:raise SystemExit(f'startup did not reach expected CD worker boundary: {required}')
if 'syscall 116' in output or 'a call to' in output:
    raise SystemExit('native binding was bypassed or unanswered')
print('Native bootstrap/timer callback reached; known original CD worker starvation reproduced.')
