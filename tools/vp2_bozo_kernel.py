"""USA-only guards for retiring SDK firmware syscall/alarm installation.

No instructions or load bytes are changed. These are port-owned native binding
declarations, deliberately separate from general SetSyscall support.
"""
import hashlib
import struct
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
MODULE_LOADER = (0x1401b8,0x1403dc)
MODULE_BUFFER_LOADER = (0x13fba8,0x13fdb0)
MODULE_BUFFER_FREE = (0x13f608,0x13f69c)
MODULE_TRANSFER = (0x137580,0x1375c0)
CACHE_SYNC = (0x137a30,0x137ad4)
COMMAND_SEND = (0x13a3f0,0x13a524)
EVENT_RANGES=((0x140b70,0x140c34),(0x148c58,0x148f88),(0x141780,0x141898),(0x141910,0x141958),(0x141980,0x1419cc))
RETIRED_RANGES = (INSTALLER, QUERY, ALARM_INSTALLER, ALARM_PAYLOAD, TIMER_INITIALIZER, TIMER_HANDLER, TIMER_START, TIMER_QUERY, TIMER_REARM,MODULE_LOADER,MODULE_BUFFER_LOADER,MODULE_BUFFER_FREE,MODULE_TRANSFER,CACHE_SYNC,COMMAND_SEND)+EVENT_RANGES
GUARDS = {
    # Retained pure software SetCmdBuffer/Add/Remove records consumed by replies.
    (0x13a2f8,0x13a3ec): 'f54416cbebce4a5d1d6ab3bf8db46d753465896633417268653373c7bbb51d21',
    COMMAND_SEND: '0a2c363e48d8b736e9b79758082641db6925e79deebb01c3d8d40db3f58d6a28',
    CACHE_SYNC: 'c0f86abc89f08902f14f584e2bb86fffdab9d4db9bf98b74698d5036480df936',
    MODULE_TRANSFER: '1f3df3b2ba20801b1d79474f0a30f158f3bc5b4eb77957a38b0c5893b27e7d6e',
    MODULE_BUFFER_LOADER: '2f7a0452693c9801e8544d03dfb9e6fc1cde09eb463a0b03a8076f02abcc611b',
    MODULE_BUFFER_FREE: 'd5e8136db76fc30774931f6ae1358a80bc0e2220eb429cda75a529e301a26395',
    MODULE_LOADER: '2d1ad203a4d7b5517bc67b88aa84038f3e62d7573c49de70f73822d807dbcc54',
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
    lo,hi=0x154f90,0x155030
    s=next(s for s in segments if s[0]==1 and s[2]<=lo<hi<=s[2]+s[4]);at=s[1]+lo-s[2]
    if hashlib.sha256(data[at:at+hi-lo]).hexdigest()!='747b2e550f56b251db82e0140e456c407e31c6d7a4602dc99b5185f3f120da13':
        raise ValueError('USA camera method vtable signature mismatch')
    lo,hi=0x11d4e0,0x11d7c0
    segment=next(s for s in segments if s[0]==1 and s[2]<=lo<hi<=s[2]+s[4])
    at=segment[1]+lo-segment[2]
    if hashlib.sha256(data[at:at+hi-lo]).hexdigest()!='ed0f34ec520d53d9a7802609a5ec468c11e60c96de07a4de4811480b3fd04085':
        raise ValueError('USA worker entry signature mismatch')
    entries=[dict(Name='worker_entry_0011d4e0',Start=hex(lo),End=hex(hi))]
    # The retail thread-object vtable at 0x151d70 points at two omitted C++
    # adjustor thunks. Each subtracts four from this and tail-calls a known body.
    lo,hi=0x107e10,0x107e30
    segment=next(s for s in segments if s[0]==1 and s[2]<=lo<hi<=s[2]+s[4])
    at=segment[1]+lo-segment[2]
    if hashlib.sha256(data[at:at+hi-lo]).hexdigest()!='d2658e525d1907be0e883232be1943cca2123ecee4b2a337007990c1ee0a5a17':
        raise ValueError('USA thread adjustor thunk signature mismatch')
    entries += [dict(Name=f'thread_adjustor_{pc:08x}',Start=hex(pc),End=hex(pc+8))
                for pc in (0x107e10,0x107e20)]
    lo,hi=0x1013d0,0x1013f0
    segment=next(s for s in segments if s[0]==1 and s[2]<=lo<hi<=s[2]+s[4]);at=segment[1]+lo-segment[2]
    if hashlib.sha256(data[at:at+hi-lo]).hexdigest()!='5cfeaf677d3aec8432a0113e4a053a1254c81a070eede39a5a72abff329e98aa':
        raise ValueError('USA shared object constructor signature mismatch')
    entries.append(dict(Name='object_constructor_001013d0',Start=hex(lo),End=hex(0x1013ec)))
    lo,hi=0x1010a0,0x1010e0
    segment=next(s for s in segments if s[0]==1 and s[2]<=lo<hi<=s[2]+s[4]);at=segment[1]+lo-segment[2]
    if hashlib.sha256(data[at:at+hi-lo]).hexdigest()!='03419fe90b5dbac2fef72a059dea794235ed0a708b7ea3147d539588febdec3e':
        raise ValueError('USA object accessor signatures mismatch')
    entries += [dict(Name=f'object_accessor_{pc:08x}',Start=hex(pc),End=hex(pc+12)) for pc in (0x1010a0,0x1010b0)]
    return entries


def native_game_sites(data):
    # This one value only feeds the boot RNG initializer's constant mix. Replace
    # its entropy producer, retaining the original mix, call and checkpoint.
    _,segments,_=elf_parts(data)
    lo,hi=0x1003bc,0x1003dc
    s=next(s for s in segments if s[0]==1 and s[2]<=lo<hi<=s[2]+s[4])
    at=s[1]+lo-s[2]
    if hashlib.sha256(data[at:at+hi-lo]).hexdigest()!='2b2969c890711c912a7d36650936ac83400288ab4461fd230ded36c5eb3c7f17':
        raise ValueError('USA boot RNG input signature mismatch')
    frame_adaptations(data)
    lo,hi=0x11f5a0,0x11f638
    s=next(s for s in segments if s[0]==1 and s[2]<=lo<hi<=s[2]+s[4])
    at=s[1]+lo-s[2]
    if hashlib.sha256(data[at:at+hi-lo]).hexdigest()!='b51e65c468e6ccfbc18ebe97e73361841f296da61c34ec5d5dafae384360053c':
        raise ValueError('USA game frame-wait signature mismatch')
    return 'native 0x1003c4\nnative 0x11fb50\nnative 0x11f62c\n'


def frame_adaptations(data):
    # Retain the game's complete frame-event body and its logical consumers.
    # Native parity replaces the producer directly, so its CSR extraction retires.
    _,segments,_=elf_parts(data)
    lo,hi=0x11fb40,0x11fc88
    s=next(s for s in segments if s[0]==1 and s[2]<=lo<hi<=s[2]+s[4])
    at=s[1]+lo-s[2]
    if hashlib.sha256(data[at:at+hi-lo]).hexdigest()!='d2442643e1005fea4dd24cbef45b3adeacb0c44a804245cedbc3cd327234a32f':
        raise ValueError('USA frame-event body signature mismatch')
    return '0011fb54 0002137a 00000000\n'


def bootstrap_tables(data):
    _,segments,_=elf_parts(data)
    lo,hi=0x145d98,0x145df0
    s=next(s for s in segments if s[0]==1 and s[2]<=lo<hi<=s[2]+s[4])
    at=s[1]+lo-s[2];code=data[at:at+hi-lo]
    if hashlib.sha256(code).hexdigest()!='16c9822fd10f6891df52e2a678e29a20f2905b41def6afba18327c3bfe7bedba':
        raise ValueError('USA startup arithmetic source mismatch')
    pairs=list(struct.iter_unpack('<II',code));constants=[lower for lower,_ in pairs[:8]]
    return b'VPBK'+struct.pack('<I8I',32,*constants[0::2],*constants[1::2])


def bootstrap_adaptations(data):
    _,segments,_=elf_parts(data)
    for lo,hi,expected in (
        (0x10087c,0x100908,'86abe7b382ebff204ec2cb2a3a01980129b566976236223b5338b03238727de5'),
        (0x100908,0x100930,'8b1dc3b6811d697d0d36c0440bbe304445f03b33f639facae842298b4d9806f1')):
        s=next(s for s in segments if s[0]==1 and s[2]<=lo<hi<=s[2]+s[4]);at=s[1]+lo-s[2]
        if hashlib.sha256(data[at:at+hi-lo]).hexdigest()!=expected:
            raise ValueError('USA startup bridge source mismatch')
    bootstrap_tables(data)
    result=[]
    for pc,replacement in ((0x10087c,0x08040235),(0x100880,0),
                           (0x1008b8,0x08040235),(0x1008bc,0),(0x1008e4,0),
                           (0x100908,0x0804024c),(0x10090c,0)):
        s=next(s for s in segments if s[0]==1 and s[2]<=pc<pc+4<=s[2]+s[4]);at=s[1]+pc-s[2]
        word=int.from_bytes(data[at:at+4],'little')
        result.append(f'{pc:08x} {word:08x} {replacement:08x}\n')
    return ''.join(result)


def overlay_adaptations(name,data):
    if name!='mod001':return ''
    _,segments,_=elf_parts(data)
    for lo,hi,expected in (
        (0x249720,0x24a438,'10c8d5e49e540decaa624ca4ea5f9e5aeee7cc23f9acf910d53cbe623ad1484e'),
        (0x202150,0x202200,'2056326076193b416411c06ca8c0b005e58038dd2b237625a6242ce3d0b661da'),
        (0x29e230,0x29e310,'62230c5e1d41b56528bb4de238ce9919fef2df3c7043e5afa91f54b1b1dce634'),
        (0x29e0b0,0x29e1c4,'4badfc2bd23ca38b95965aa45332f82c6efd9655b77f0d7b9344126467e8745d'),
        (0x312280,0x313100,'a834d66f34036f87f4061c9e5c2d623b29ef77f6fd6394b765333d0f72c45617'),
        (0x314138,0x31413c,'e7f5662273401b756d9f64b866d1a7df7e95c248c34f9b0cad011eb84757f186'),
        (0x217550,0x2175c0,'bf29f73fdf9756c87ee0b7b11aaccb586cb29e0fb0653934452e96586c48b694'),
        (0x2494c0,0x249720,'c017bf6d329e88332552da7c1f5949a763f4900862bc056f18fbd56b1a9848df'),
        (0x249270,0x249390,'5ddcd40fb0819990a6319eb8774b68d84ced2390068859deb276a6acf162c1ce'),
        (0x2c3c10,0x2c4378,'06537ab19eb16967c9643170fcf6e3a0334037ac864618129abfd696bd793ab5'),
        (0x216730,0x216780,'d863942f37e0b5485e89ddbb09d7f46b7e9112e833fbbbf3b19295fb6ec915dd'),
        (0x333640,0x333650,'806ce2a8243f8efbbe73d07c33d5ede4d06c673cc60521522982122459c89c76'),
        (0x333650,0x333d60,'1af5ec727cc7f94f98f183a70089d92938c5f386748cc07562e682558fede684'),
        (0x29e1d0,0x29e22c,'5a8f6625f0c067de0c3cc1c74f1e25deb76a988f034f3a7ee7fc6d82989f5834')):
        s=next(s for s in segments if s[0]==1 and s[2]<=lo<hi<=s[2]+s[4]);at=s[1]+lo-s[2]
        if hashlib.sha256(data[at:at+hi-lo]).hexdigest()!=expected:
            raise ValueError('USA graphics setup body signature mismatch')
    # Native callbacks/jobs need no PS2 global enable or channel-nine routing
    # bit. Preserve every game allocation, record and callback registration.
    return ('00249830 ac22e000 00000000\n'
            '002021c4 3c011001 0808087a\n'
            '002021c8 2402fffd 00000000\n'
            '0029e210 4a00fc3e 00000000\n'
            '0029e214 4b1f043d 00000000\n'
            # Only this program-install enqueue retires. Its 220 instruction
            # pairs are code/default constants, with no execution or data draw;
            # its completion method is empty. Retain object initialization and
            # the delay slot. Do not submit/complete a fictitious native job.
            '0029e2fc 0c0807c8 00000000\n'
            '0029e1a0 a0820050 00000000\n'
            '0029e1ac 0c0807c8 00000000\n'
            '002c3cb4 8c220000 00000000\n'
            '002c3cb8 10000015 080b0f48\n')


def overlay_native_sites(name,data):
    if name!='mod001':return ''
    overlay_adaptations(name,data)
    return 'native 0x29e1a0\nnative 0x2c3cb4\n'
