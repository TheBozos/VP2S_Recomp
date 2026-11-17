"""Virtual Xbox 360 pad (ViGEmBus) for driving PCSX2 without window focus.

PCSX2 reads it as an SDL game controller in the background, so route pad
scripts replay while the user keeps using the PC: no key injection.
The ViGEmBus driver must be installed; the client DLL path comes from
VPAD_VIGEM_CLIENT (ViGEmClient.dll, x64).
"""
import ctypes, os
from ctypes import c_void_p, c_uint, c_ushort, c_ubyte, c_short

class XUSB_REPORT(ctypes.Structure):
    _fields_ = [('wButtons', c_ushort), ('bLeftTrigger', c_ubyte), ('bRightTrigger', c_ubyte),
                ('sThumbLX', c_short), ('sThumbLY', c_short), ('sThumbRX', c_short), ('sThumbRY', c_short)]

# Pad script names -> XUSB buttons (PCSX2 binds FaceSouth=Cross, FaceEast=Circle, ...).
BUTTONS = {'UP': 0x0001, 'DOWN': 0x0002, 'LEFT': 0x0004, 'RIGHT': 0x0008, 'START': 0x0010, 'SELECT': 0x0020,
           'L3': 0x0040, 'R3': 0x0080, 'L1': 0x0100, 'R1': 0x0200,
           'CROSS': 0x1000, 'CIRCLE': 0x2000, 'SQUARE': 0x4000, 'TRIANGLE': 0x8000}
TRIGGERS = {'L2', 'R2'}

class VirtualPad:
    def __init__(self, dll=None):
        dll = dll or os.environ.get('VPAD_VIGEM_CLIENT')
        if not dll or not os.path.exists(dll):
            raise RuntimeError('set VPAD_VIGEM_CLIENT to ViGEmClient.dll (x64)')
        self.lib = lib = ctypes.CDLL(dll)
        lib.vigem_alloc.restype = c_void_p
        lib.vigem_connect.argtypes = (c_void_p,); lib.vigem_connect.restype = c_uint
        lib.vigem_target_x360_alloc.restype = c_void_p
        lib.vigem_target_add.argtypes = (c_void_p, c_void_p); lib.vigem_target_add.restype = c_uint
        lib.vigem_target_x360_update.argtypes = (c_void_p, c_void_p, XUSB_REPORT); lib.vigem_target_x360_update.restype = c_uint
        lib.vigem_target_remove.argtypes = (c_void_p, c_void_p); lib.vigem_target_remove.restype = c_uint
        lib.vigem_target_free.argtypes = (c_void_p,); lib.vigem_disconnect.argtypes = (c_void_p,); lib.vigem_free.argtypes = (c_void_p,)
        self.client = lib.vigem_alloc()
        if lib.vigem_connect(self.client) != 0x20000000:
            raise RuntimeError('ViGEmBus not reachable')
        self.target = lib.vigem_target_x360_alloc()
        if lib.vigem_target_add(self.client, self.target) != 0x20000000:
            raise RuntimeError('cannot plug the virtual pad')
        self.set(())

    def set(self, names):
        r = XUSB_REPORT()
        for n in names:
            if n in TRIGGERS:
                setattr(r, 'bLeftTrigger' if n == 'L2' else 'bRightTrigger', 255)
            else:
                r.wButtons |= BUTTONS[n]
        if self.lib.vigem_target_x360_update(self.client, self.target, r) != 0x20000000:
            raise RuntimeError('virtual pad update failed')

    def close(self):
        if self.target:
            self.set(())
            self.lib.vigem_target_remove(self.client, self.target); self.lib.vigem_target_free(self.target)
            self.lib.vigem_disconnect(self.client); self.lib.vigem_free(self.client)
            self.target = None

    def __enter__(self): return self
    def __exit__(self, *a): self.close()
