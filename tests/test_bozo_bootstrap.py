"""Bounded headless integration through native timer/CD callback setup.

No window/input options or card directory are passed. The runner uses fresh
in-memory cards. The first unsupported native service module remains a hard stop;
this test qualifies that boundary only, not a playable route or visual parity.
"""
import subprocess
import sys
import os

result=subprocess.run([sys.argv[1],'--iso',sys.argv[2],'--frames','1'],
                      capture_output=True,text=True,timeout=30,
                      env={**os.environ,'VP2S_BOZO_TRACE_CALLS':'1','VP2S_BOZO_SEED':'1'})
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
                 '[vp2s:call] pc=128fb8 ra=11f1e8',
                 '[vp2s:call] pc=128de0 ra=11f208',
                 '[vp2s:call] pc=12a330 ra=11d718',
                 'native pad/card service owns sio2man.irx id=1',
                 'native pad/card service owns sio2d.irx id=2',
                 'compiled native service owns Dbc_Manager.irx id=3',
                 'compiled native service owns ds2o_d.irx id=4',
                 'compiled native service owns Sound_Device_Library.irx id=5',
                 'unported native service module: Csm_.irx']:
    if required not in output:raise SystemExit(f'startup did not reach expected CD worker boundary: {required}')
if 'syscall 116' in output or 'a call to' in output:
    raise SystemExit('native binding was bypassed or unanswered')
if 'pc=137220 a0=ffffffff result=-408' in output:
    raise SystemExit('the retired CD private worker unexpectedly ran')
print('Native bootstrap/timer/CD/DBC/controller/libsd initialization reached; Tri-Ace Csm remains unsupported.')
