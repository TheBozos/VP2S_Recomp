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
EVENT_RANGES=((0x140b70,0x140c34),(0x148c58,0x148f88),(0x141780,0x141898),(0x141910,0x141958),(0x141980,0x1419cc))
RETIRED_RANGES = (INSTALLER, QUERY, ALARM_INSTALLER, ALARM_PAYLOAD, TIMER_INITIALIZER, TIMER_HANDLER, TIMER_START, TIMER_QUERY, TIMER_REARM)+EVENT_RANGES
GUARDS = {
    (0x137280,0x1372a0): '56d370ced612e106f16a1061ab16f2b871adc7a8fd989a6a5db7c3bf2fd09f8a',
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
    (0x142090, 0x142114): '75c8886e0ff464f9aafdc4f95a9d3a43dc70ffdbf6682c8fffe30ba9195e0939',
    (0x142118, 0x142150): '9bae4d737c51e5d55a29b4d54d822e495daa94202b70d7914eebcc9d83da0d93',
    (0x142e80, 0x142ecc): '0442fda8aa67f2843c3fd3b1e980d95273160bf40796a9bfa01d4d294ebecf47',
    (0x1412a0, 0x1412f0): '018be3a7aa3a1a8d9fd3e43aaead80a463f13bb8b7a47d0a4f5a0e9293a12cb5',
    (0x14a238, 0x14a258): '30a8247e2fc28ad1a6bec0a7411b80aac0dbda1ceaa0d09570f71e33c5649d11',
    TIMER_START: '3d2d681aabb1078a349c96bef48224bc670e20106442f49f62933bc54347c036',
    TIMER_QUERY: '25ac1b70aae992486cebb5d405f2edaf181187052a4a02ad0278fd2a327da1ab',
    TIMER_REARM: '450fb96f23fc04e55a2188ec86ad62cad9072fb70cd96a18f07769d78d7d34c0',
    (0x140b70,0x140c34): '886f21617db013d0f6884104f8a94540a5a5e1ccd1e128db4bfffce9d9c13eb6',
    (0x148c58,0x148f88): '3812e2924e67b94a294d790deee44b51e4ee7b6eb2bb9aa9899380ce006cd5a2',
    (0x148fd0,0x149018): '07d4778b2e7b7da6cf4a1889096337bff99bf24a7d21668f9dedeceb9348b456',
    (0x140b08,0x140b70): 'e5759de7a93e89145c691b41dfb6bef6c587e1fbbb582e126f261eb6b3c6a338',
    (0x141780,0x141898): '43cfc177016ecbab99f018bbe39327245b021c1f88545704018d559733939fc0',
    (0x141910,0x141958): '47da7bf63fc6a808dd63d3c16b87f93d4ac14aa104b35b250b38df67f964eaae',
    (0x141980,0x1419cc): 'e0617b1a7a3fcd18cd793cbdb6e8712c4562cf243b2b5adc8b038f8ae32bcf11',
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


def game_adaptations(data):
    # Native frame timing owns elapsed frames. Retire the main routine's two
    # HBLANK-counter stores only; retain its adjacent game magic/data store and
    # the call/checkpoint/delay-slot structure. Unported readers still fault.
    _,segments,_=elf_parts(data)
    lo,hi=0x1002c0,0x100304
    segment=next(s for s in segments if s[0]==1 and s[2]<=lo<hi<=s[2]+s[4])
    start=segment[1]+lo-segment[2]
    expected='4593352447efec5a5e14d31c108e37f12bbd014fe7036549f96113618883b967'
    if hashlib.sha256(data[start:start+hi-lo]).hexdigest()!=expected:
        raise ValueError('USA game timing initialization signature mismatch')
    result=[]
    for pc,word in [(0x1002f4,0xac220010),(0x100300,0xac200000)]:
        at=segment[1]+pc-segment[2]
        if int.from_bytes(data[at:at+4],'little')!=word:
            raise ValueError(f'game timing store mismatch: {pc:x}')
        result.append(f'{pc:08x} {word:08x} 00000000\n')
    return ''.join(result)


def game_entries(data):
    # The thread object's vtable dispatches here from 0x119750. Ghidra exported
    # only the tail labels 0x11d770..0x11d7c0, omitting the actual entry/prologue.
    # Its source extent ends at the next independent prologue, 0x11d7c0.
    _,segments,_=elf_parts(data)
    lo,hi=0x11d4e0,0x11d7c0
    segment=next(s for s in segments if s[0]==1 and s[2]<=lo<hi<=s[2]+s[4])
    at=segment[1]+lo-segment[2]
    if hashlib.sha256(data[at:at+hi-lo]).hexdigest()!='ed0f34ec520d53d9a7802609a5ec468c11e60c96de07a4de4811480b3fd04085':
        raise ValueError('USA worker entry signature mismatch')
    return [dict(Name='worker_entry_0011d4e0',Start=hex(lo),End=hex(hi))]
