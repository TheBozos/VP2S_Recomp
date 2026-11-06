"""USA-only guards for retiring SDK firmware syscall/alarm installation.

No instructions or load bytes are changed. These are port-owned native binding
declarations, deliberately separate from general SetSyscall support.
"""
import hashlib
from vp2_bozo_prepare import elf_parts

INSTALLER = (0x141448, 0x141544)
QUERY = (0x141410, 0x141444)
ALARM_INSTALLER = (0x141a38, 0x141b0c)
ALARM_PAYLOAD = (0x149a90, 0x14a1d0)
TIMER_INITIALIZER = (0x141bb0, 0x141d04)
TIMER_HANDLER = (0x142150, 0x1423f4)
TIMER_START = (0x141df0, 0x141e80)
TIMER_QUERY = (0x1423f8, 0x142444)
TIMER_REARM = (0x141f00, 0x142090)
RETIRED_RANGES = (INSTALLER, QUERY, ALARM_INSTALLER, ALARM_PAYLOAD, TIMER_INITIALIZER, TIMER_HANDLER, TIMER_START, TIMER_QUERY, TIMER_REARM)
GUARDS = {
    (0x141378, 0x141444): 'e0986ae7dae0f8e7c70713eb78d9c5f8fc026874de14dbcfbfca790c983be506',
    INSTALLER: 'e3c497dbca0fa7aa92e3ec912159cbac2c0e5c24819b259ac207a22e8c0fbdf4',
    (0x141558, 0x1415ac): 'f99915ebc05eeca2deb3b798fde342e27ee457084bc30313a685c6ae54834328',
    (0x1492a8, 0x1492c0): 'c0a2bc71d842355d9d060d18967d916f5d3e776a0cf84296253406ca5f7a489a',
    ALARM_INSTALLER: '0a4562fbdf8a3c607e98899a67d606175b3fa4e5fd6a61ab794760c1081e35ff',
    (0x1419d0, 0x141a38): '22955879613d98919c265f48c4daeea1124abfc46f4cdc2a23557a0439134c9f',
    ALARM_PAYLOAD: '91fea21c5fbac87ffaa363b2810520e72a916b9dcd79f35033036a548e19c914',
    (0x14a1d0, 0x14a1f8): '1c2f1fe1f282c987780e310df26346b33105c89f7d6fea87a537cd0907b60def',
    (0x14a1f8, 0x14a238): '5e317d6f7a1abfb8f1b130d03c30622f3d5d3ab4842d39507991dcc7af7813be',
    TIMER_INITIALIZER: '3f10a54756d65ac0aed89a43e8ff49241e06b6bf3e892bcb523909ed87622a6a',
    TIMER_HANDLER: '86a5dd2150d80e0dc357fcd2b4183b8861de75d2355864eb6c5a50a239270e32',
    (0x142e80, 0x142ecc): '0442fda8aa67f2843c3fd3b1e980d95273160bf40796a9bfa01d4d294ebecf47',
    (0x1412a0, 0x1412f0): '018be3a7aa3a1a8d9fd3e43aaead80a463f13bb8b7a47d0a4f5a0e9293a12cb5',
    (0x14a238, 0x14a258): '30a8247e2fc28ad1a6bec0a7411b80aac0dbda1ceaa0d09570f71e33c5649d11',
    TIMER_START: '3d2d681aabb1078a349c96bef48224bc670e20106442f49f62933bc54347c036',
    TIMER_QUERY: '25ac1b70aae992486cebb5d405f2edaf181187052a4a02ad0278fd2a327da1ab',
    TIMER_REARM: '450fb96f23fc04e55a2188ec86ad62cad9072fb70cd96a18f07769d78d7d34c0',
}


def bindings(data):
    _, segments, sections = elf_parts(data)
    for (lo, hi), expected in GUARDS.items():
        matches = [s for s in segments if s[0] == 1 and s[2] <= lo < hi <= s[2]+s[4]]
        if len(matches) != 1:
            raise ValueError(f'native syscall-bootstrap guard not image-backed: {lo:x}')
        segment = matches[0]
        start = segment[1]+lo-segment[2]
        if hashlib.sha256(data[start:start+hi-lo]).hexdigest() != expected:
            raise ValueError(f'USA syscall-bootstrap signature mismatch: {lo:x}')
    sites = sorted({pc for lo,hi in RETIRED_RANGES for pc in range(lo,hi,4)})
    # Every possible interior entry is a rejecting out-call. Native-instruction
    # guards also protect generated resume paths that bypass the call table.
    declarations = []
    for pc in sites:
        declarations.append(f'stub 0x{pc:x}\n')
        segment = next(s for s in segments if s[0] == 1 and s[2] <= pc < s[2]+s[4])
        offset = segment[1]+pc-segment[2]
        word = int.from_bytes(data[offset:offset+4], 'little')
        opcode = word >> 26
        control = opcode in (1, 2, 3, 4, 5, 6, 7, 20, 21, 22, 23) or (opcode == 0 and word & 63 in (8, 9))
        code = any(s[2]&6 == 6 and s[3]<=pc and pc+4<=s[3]+s[5] for s in sections)
        if code and not control:
            declarations.append(f'native 0x{pc:x}\n')
    return ''.join(declarations)
