"""Replay a Bozo route pad script in PCSX2 through a virtual pad, capturing
the window with PrintWindow: no focus, no key injection (the user keeps the PC).

PCSX2 Pad 1 is bound to SDL-0 (the virtual XInput pad) besides the keyboard.
Frames are real time (1/59.94 s) from launch, shifted by --offset (PCSX2 boot).
  python tools/pcsx2_vpad_replay.py SCRIPT OUTDIR --until F [--offset 250]
         [--capture-from A --capture-to B --capture-every N]
"""
import argparse, ctypes, os, subprocess, sys, time
from ctypes import wintypes
sys.path.insert(0, os.path.dirname(__file__))
from vpad import VirtualPad
from pad_script_to_p2m2 import parse
from PIL import Image

PCSX2 = r'C:\Program Files\PCSX2\pcsx2-qt.exe'
ISO = r'D:\VP2S_Recomp\Valkyrie Profile 2 - Silmeria (USA).iso'
FRAME = 1 / 59.94
user32, gdi32 = ctypes.windll.user32, ctypes.windll.gdi32

def window_of(pid):
    found = []
    @ctypes.WINFUNCTYPE(wintypes.BOOL, wintypes.HWND, wintypes.LPARAM)
    def cb(h, _):
        p = wintypes.DWORD(); user32.GetWindowThreadProcessId(h, ctypes.byref(p))
        if p.value == pid and user32.IsWindowVisible(h) and user32.GetWindowTextLengthW(h) > 0: found.append(h)
        return True
    user32.EnumWindows(cb, 0)
    return found[0] if found else None

def capture(h, path):
    r = wintypes.RECT(); user32.GetClientRect(h, ctypes.byref(r)); w, hh = r.right, r.bottom
    if w <= 0 or hh <= 0: return False
    hdc = user32.GetDC(h); mdc = gdi32.CreateCompatibleDC(hdc); bmp = gdi32.CreateCompatibleBitmap(hdc, w, hh)
    gdi32.SelectObject(mdc, bmp); user32.PrintWindow(h, mdc, 3)  # PW_CLIENTONLY | PW_RENDERFULLCONTENT
    class BMI(ctypes.Structure):
        _fields_ = [('biSize', wintypes.DWORD), ('biWidth', wintypes.LONG), ('biHeight', wintypes.LONG), ('biPlanes', wintypes.WORD),
                    ('biBitCount', wintypes.WORD), ('biCompression', wintypes.DWORD), ('rest', wintypes.DWORD * 6)]
    bi = BMI(); bi.biSize = 40; bi.biWidth = w; bi.biHeight = -hh; bi.biPlanes = 1; bi.biBitCount = 32
    buf = ctypes.create_string_buffer(w * hh * 4)
    gdi32.GetDIBits(mdc, bmp, 0, hh, buf, ctypes.byref(bi), 0)
    gdi32.DeleteObject(bmp); gdi32.DeleteDC(mdc); user32.ReleaseDC(h, hdc)
    Image.frombuffer('RGBA', (w, hh), buf, 'raw', 'BGRA', 0, 1).convert('RGB').save(path)
    return True

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('script'); ap.add_argument('outdir')
    ap.add_argument('--until', type=int, required=True)
    ap.add_argument('--offset', type=int, default=250)
    ap.add_argument('--capture-from', type=int, default=0); ap.add_argument('--capture-to', type=int, default=None)
    ap.add_argument('--capture-every', type=int, default=30)
    a = ap.parse_args()
    os.makedirs(a.outdir, exist_ok=True)
    presses = [p for p in parse(open(a.script, encoding='utf-8').read()) if p[0] < a.until]
    events = []
    for start, duration, name in presses:
        events += [(start, name, True), (start + duration, name, False)]
    to = a.capture_to if a.capture_to is not None else a.until
    events += [(f, None, None) for f in range(a.capture_from, to + 1, a.capture_every)]
    events.sort(key=lambda e: (e[0], e[1] is None))
    with VirtualPad() as pad:
        time.sleep(1)
        proc = subprocess.Popen([PCSX2, '-fastboot', '--', ISO]); t0 = time.perf_counter()
        held, h = set(), None
        try:
            for frame, name, down in events:
                due = t0 + (frame + a.offset) * FRAME
                while (now := time.perf_counter()) < due: time.sleep(min(0.005, due - now))
                if name is None:
                    h = h or window_of(proc.pid)
                    if h: capture(h, os.path.join(a.outdir, f'pcsx2_{frame:06d}.png'))
                    continue
                (held.add if down else held.discard)(name); pad.set(held)
        finally:
            proc.kill()
    print('done', a.outdir)

if __name__ == '__main__':
    main()
