"""Bounded headless integration: firmware retirement reaches timer initialization.

No window/input options or card directory are passed. The runner uses fresh
in-memory cards. The separate unported timer initializer must remain a hard
failure. Retiring InitAlarm's installer does not qualify runtime alarm APIs.
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
if 'unported device read 10001010 at pc=141c80' not in output:
    raise SystemExit('startup did not stop at the separate audited timer initialization boundary')
if 'syscall 116' in output or 'a call to' in output:
    raise SystemExit('native binding was bypassed or unanswered')
print('Both firmware installers retired; separate unsupported timer initialization rejected.')
