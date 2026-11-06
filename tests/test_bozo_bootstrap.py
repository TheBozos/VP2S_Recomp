"""Bounded headless integration through worker startup to pending SDK deadlines.

No window/input options or card directory are passed. The runner uses fresh
in-memory cards. A pending SDK timer list remains a hard failure until its
deadline/callback path is ported. Installer retirement does not qualify alarms.
"""
import subprocess
import sys

result=subprocess.run([sys.argv[1],'--iso',sys.argv[2],'--frames','1'],
                      capture_output=True,text=True,timeout=30)
output=result.stdout+result.stderr
print(output,end='')
if result.returncode!=1:
    raise SystemExit(f'expected explicit unported boundary, got {result.returncode}')
if '[vp2s:bozo] native syscall services ready; retired SDK firmware discovery' not in output:
    raise SystemExit('startup did not reach the native installer')
if '[vp2s:bozo] retired SDK alarm firmware installer; runtime alarm APIs remain unsupported' not in output:
    raise SystemExit('startup did not retire the alarm firmware installer')
if '[vp2s:bozo] native timer records initialized; runtime timer callback remains unsupported' not in output:
    raise SystemExit('startup did not reach native timer record initialization')
if '[vp2s:bozo] native SDK timer started' not in output:
    raise SystemExit('startup did not start the native SDK clock')
if '[vp2s:bozo] retired SDK event firmware installer' not in output:
    raise SystemExit('startup did not retire the audited event firmware installer')
if 'native SDK timer rearm requires pending deadline support' not in output:
    raise SystemExit('startup did not reach the pending native deadline boundary')
if 'syscall 116' in output or 'a call to' in output:
    raise SystemExit('native binding was bypassed or unanswered')
print('Native bootstrap/worker startup passed; unsupported pending deadline boundary reached.')
