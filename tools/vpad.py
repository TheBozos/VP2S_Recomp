"""Virtual DualShock 4 (ViGEmBus) for driving PCSX2 without window focus,
hidden from every other program by HidHide.

A virtual Xbox pad is global (XInput): the user's other games received the
route's inputs. A DS4 is a HID device, so HidHide can hide it from everything
except the allow-listed PCSX2. VirtualPad refuses to press anything unless
cloaking is on, PCSX2 is allow-listed and the plugged pad is on the hidden list.

The client DLL path comes from VPAD_VIGEM_CLIENT (ViGEmClient.dll, x64).
"""
import ctypes, json, os, subprocess, time
from ctypes import c_void_p, c_uint, c_ushort, c_ubyte

HIDHIDE_CLI = r'C:\Program Files\Nefarius Software Solutions\HidHide\x64\HidHideCLI.exe'
PCSX2 = r'C:\Program Files\PCSX2\pcsx2-qt.exe'
OK = 0x20000000

class DS4_REPORT(ctypes.Structure):
    _fields_ = [('bThumbLX', c_ubyte), ('bThumbLY', c_ubyte), ('bThumbRX', c_ubyte), ('bThumbRY', c_ubyte),
                ('wButtons', c_ushort), ('bSpecial', c_ubyte), ('bTriggerL', c_ubyte), ('bTriggerR', c_ubyte)]

# Pad script names -> DS4 buttons (SDL: FaceSouth=Cross, Back=Share, Start=Options, Guide=PS).
BUTTONS = {'SQUARE': 1 << 4, 'CROSS': 1 << 5, 'CIRCLE': 1 << 6, 'TRIANGLE': 1 << 7, 'L1': 1 << 8, 'R1': 1 << 9,
           'L2': 1 << 10, 'R2': 1 << 11, 'SELECT': 1 << 12, 'START': 1 << 13, 'L3': 1 << 14, 'R3': 1 << 15}
SPECIAL = {'GUIDE': 1}
DPAD = {(1, 0, 0, 0): 0, (1, 0, 0, 1): 1, (0, 0, 0, 1): 2, (0, 1, 0, 1): 3, (0, 1, 0, 0): 4,
        (0, 1, 1, 0): 5, (0, 0, 1, 0): 6, (1, 0, 1, 0): 7}   # (up, down, left, right) -> hat; else 8 = none

def hidhide(*args):
    out = subprocess.run([HIDHIDE_CLI, *args], capture_output=True, text=True, timeout=30)
    if out.returncode: raise RuntimeError(f'HidHideCLI {args} failed: {out.stderr.strip()}')
    return out.stdout

def gaming_paths():
    text = hidhide('--dev-gaming').strip()
    paths = set()
    for group in json.loads(text) if text else []:
        for dev in group.get('devices', []):
            paths.add(dev.get('deviceInstancePath', ''))
    return {p for p in paths if p}

def hidden_paths():
    return {line.split('"')[1] for line in hidhide('--dev-list').splitlines() if '"' in line}

class VirtualPad:
    def __init__(self, dll=None):
        if 'cloak-on' not in hidhide('--cloak-state'):
            raise RuntimeError('HidHide cloaking is off: the virtual pad would reach every game')
        if PCSX2.lower() not in hidhide('--app-list').lower():
            raise RuntimeError('PCSX2 is not allow-listed in HidHide')
        dll = dll or os.environ.get('VPAD_VIGEM_CLIENT')
        if not dll or not os.path.exists(dll):
            raise RuntimeError('set VPAD_VIGEM_CLIENT to ViGEmClient.dll (x64)')
        self.lib = lib = ctypes.CDLL(dll)
        lib.vigem_alloc.restype = c_void_p
        lib.vigem_connect.argtypes = (c_void_p,); lib.vigem_connect.restype = c_uint
        lib.vigem_target_ds4_alloc.restype = c_void_p
        lib.vigem_target_add.argtypes = (c_void_p, c_void_p); lib.vigem_target_add.restype = c_uint
        lib.vigem_target_ds4_update.argtypes = (c_void_p, c_void_p, DS4_REPORT); lib.vigem_target_ds4_update.restype = c_uint
        lib.vigem_target_remove.argtypes = (c_void_p, c_void_p); lib.vigem_target_remove.restype = c_uint
        lib.vigem_target_free.argtypes = (c_void_p,); lib.vigem_disconnect.argtypes = (c_void_p,); lib.vigem_free.argtypes = (c_void_p,)
        self.target = None; self.armed = False
        before = gaming_paths()
        self.client = lib.vigem_alloc()
        if lib.vigem_connect(self.client) != OK:
            raise RuntimeError('ViGEmBus not reachable')
        self.target = lib.vigem_target_ds4_alloc()
        if lib.vigem_target_add(self.client, self.target) != OK:
            raise RuntimeError('cannot plug the virtual pad')
        try:
            # The new HID device is the virtual pad; it must be hidden before any press.
            new = set()
            for _ in range(50):
                new = gaming_paths() - before
                if new: break
                time.sleep(0.1)
            if not new: raise RuntimeError('virtual pad HID device not found')
            for path in new - hidden_paths(): hidhide('--dev-hide', path)
            if new - hidden_paths(): raise RuntimeError(f'virtual pad not hidden: {new}')
            self.paths = new; self.armed = True
            self.set(())
        except Exception:
            self.close(); raise

    def set(self, names):
        if not self.armed: raise RuntimeError('virtual pad is not hidden')
        r = DS4_REPORT(0x80, 0x80, 0x80, 0x80, 8, 0, 0, 0)
        hat = tuple(int(n in names) for n in ('UP', 'DOWN', 'LEFT', 'RIGHT'))
        r.wButtons = DPAD.get(hat, 8)
        for n in names:
            if n in BUTTONS: r.wButtons |= BUTTONS[n]
            if n in SPECIAL: r.bSpecial |= SPECIAL[n]
            if n == 'L2': r.bTriggerL = 255
            if n == 'R2': r.bTriggerR = 255
        if self.lib.vigem_target_ds4_update(self.client, self.target, r) != OK:
            raise RuntimeError('virtual pad update failed')

    def close(self):
        if self.target:
            if self.armed: self.set(())
            self.armed = False
            self.lib.vigem_target_remove(self.client, self.target); self.lib.vigem_target_free(self.target)
            self.lib.vigem_disconnect(self.client); self.lib.vigem_free(self.client)
            self.target = None

    def __enter__(self): return self
    def __exit__(self, *a): self.close()
