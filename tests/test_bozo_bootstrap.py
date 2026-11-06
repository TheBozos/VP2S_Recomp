"""Bounded headless integration: native discovery retirement reaches InitAlarm.

No window/input options or card directory are passed. The runner uses fresh
in-memory cards. The next unported timer boundary must remain a hard failure.
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
if 'unported device read 10001810 at pc=141a38' not in output:
    raise SystemExit('startup did not stop at the audited next InitAlarm boundary')
if 'syscall 116' in output or 'a call to' in output:
    raise SystemExit('native binding was bypassed or unanswered')
print('Native installer reached; next unsupported InitAlarm boundary rejected.')
