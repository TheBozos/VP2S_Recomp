"""Check that the virtual DS4 is invisible to programs other than PCSX2.

Run from a normal (not allow-listed) Python: plugs the pad through
VirtualPad (which hides it), then tries what a game would do:
  - open the pad's HID device: must fail;
  - read XInput slots: no slot may appear while the pad is plugged.
Exit code 0 only when both hold.
  python tools/vpad_isolation_check.py
"""
import ctypes, json, os, sys
from ctypes import wintypes
sys.path.insert(0, os.path.dirname(__file__))
from vpad import VirtualPad, hidhide

k32 = ctypes.WinDLL('kernel32', use_last_error=True)
k32.CreateFileW.restype = wintypes.HANDLE
k32.CreateFileW.argtypes = (wintypes.LPCWSTR, wintypes.DWORD, wintypes.DWORD, ctypes.c_void_p,
                            wintypes.DWORD, wintypes.DWORD, wintypes.HANDLE)
INVALID = wintypes.HANDLE(-1).value
xinput = ctypes.WinDLL('xinput1_4')

def xinput_slots():
    state = (ctypes.c_ubyte * 16)()
    return {i for i in range(4) if xinput.XInputGetState(i, state) == 0}

def links(paths):
    text = hidhide('--dev-gaming').strip()
    return [d['symbolicLink'] for g in json.loads(text) for d in g['devices'] if d['deviceInstancePath'] in paths]

def main():
    before = xinput_slots()
    with VirtualPad() as pad:
        ok = True
        for link in links(pad.paths):
            h = k32.CreateFileW(link, 0, 3, None, 3, 0, None)
            if h != INVALID:
                k32.CloseHandle(h); ok = False; print('VISIBLE: opened', link)
            else:
                print('hidden: open failed (error', ctypes.get_last_error(), ')', link)
        after = xinput_slots()   # no press: a failed check must not reach any game
        if after - before:
            ok = False; print('VISIBLE: new XInput slot', after - before)
        else:
            print('XInput: no new slot', sorted(after))
    print('isolated' if ok else 'NOT ISOLATED'); return 0 if ok else 1

if __name__ == '__main__':
    sys.exit(main())
