"""Bounded headless native startup integration, without a rendered-scene claim.

No window, host-input provider or card directory is used. The last scenario
uses a temporary native input script and all runs use fresh in-memory cards.
Unsupported operations remain hard stops; the longer run
qualifies its current graphics/math boundary, not a playable route or visual parity.
"""
import subprocess
import tempfile
from pathlib import Path
import sys
import os

result=subprocess.run([sys.argv[1],'--iso',sys.argv[2],'--frames','1'],
                      capture_output=True,text=True,timeout=30,
                      env={**os.environ,'VP2S_BOZO_TRACE_CALLS':'1','VP2S_BOZO_SEED':'1'})
output=result.stdout+result.stderr
print(output,end='')
if result.returncode!=0:
    raise SystemExit(f'expected first native game-frame boundary, got {result.returncode}')
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
                 'compiled native service owns Csm_.irx id=6',
                 'compiled native service owns Csd_.irx id=7',
                 'compiled native service owns Csi_.irx id=8',
                 'native pad/card service owns mcman.irx id=9',
                 'native pad/card service owns mcserv.irx id=10',
                 'frames=1 ', 'end=frame limit 1 reached']:
    if required not in output:raise SystemExit(f'startup did not reach expected CD worker boundary: {required}')
if 'syscall 116' in output or 'a call to' in output:
    raise SystemExit('native binding was bypassed or unanswered')
if 'pc=137220 a0=ffffffff result=-408' in output:
    raise SystemExit('the retired CD private worker unexpectedly ran')
deeper=subprocess.run([sys.argv[1],'--iso',sys.argv[2],'--frames','600'],
                      capture_output=True,text=True,timeout=30,
                      env={**os.environ,'VP2S_BOZO_SEED':'1','VP2S_BOZO_TRACE_QUEUES':'1'})
print(deeper.stdout+deeper.stderr,end='')
longer=deeper.stdout+deeper.stderr
if deeper.returncode!=0 or 'frames=600 ' not in longer or 'end=frame limit 600 reached' not in longer:
    raise SystemExit('longer native boot did not complete 600 logical frames')
for required in ('[vp2s:screen] native rect','captured text=997620 glyphs=171 inline=0','native layer=997530 texts=1'):
    if required not in longer:raise SystemExit(f'longer native boot bypassed native source publication: {required}')
if 'native boundary:' in longer:raise SystemExit('longer native boot reached an unexpected rejecting boundary')
print('600 native logical frames passed with texture publication, scalar glyph capture and native UI submission. Presentation and the Logun route remain unqualified.')
with tempfile.TemporaryDirectory(prefix='vp2s-native-input-') as temporary:
    script=Path(temporary)/'screen.txt';script.write_text('200:SQUARE/6',encoding='ascii')
    media=subprocess.run([sys.argv[1],'--iso',sys.argv[2],'--frames','300','--pad-script',str(script)],
                         capture_output=True,text=True,timeout=30,
                         env={**os.environ,'VP2S_BOZO_SEED':'1','VP2S_BOZO_TRACE_QUEUES':'1'})
    output=media.stdout+media.stderr;print(output,end='')
    if media.returncode!=1 or 'native VP2S MPEG provider required at 1f4590 ra=2bdf80' not in output:
        raise SystemExit('scripted startup did not reach the native overlay MPEG ownership guard')
    if 'native decoder owns prepared movie input' not in output or 'native MPEG initializer ready' not in output or 'native decoder owns original MPEG software records' not in output or 'unported device' in output:
        raise SystemExit('scripted startup bypassed native buffer ownership or reached SDK hardware code')
    if 'native callback type=4 function=2b9320' not in output:raise SystemExit('scripted startup bypassed native callback registration')
    if 'native stream type=4 channel=1' not in output:raise SystemExit('scripted startup bypassed native stream registration')
    if 'native decoder reset completed' not in output:raise SystemExit('scripted startup bypassed native decoder reset')
print('Scripted native input passes decoder and paired input reset, then rejects unowned PSS demux before its SDK/ABI body. No movie/Logun claim.')
