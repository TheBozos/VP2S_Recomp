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
    (0x15a1e0,0x15a200): '3d5516bcb133aad0302be1420e75fd0d283d1ad45aed193efeaef36368eb110f',
    (0x152060,0x152080): '82feb318e1227eae8460ec6bb489653ef560aab596c9ec7ef9a04b74df95c62a',
    (0x15ba00,0x15babc): '01c9487bea078d22ae8dea8d82ce3f48ce0d593fe75191f3598e7e805e987002',
    (0x15a450,0x15a490): '68cb068f8c4169c07e5d6a267764a66a0f6a068e09de846516b8726d30fef617',
    (0x15bc00,0x15bc9c): '72c0184afee23c2710a75366952b45666767799be208e4c2d840b5718f28da27',
    (0x120a80,0x120c40): '40434feee6a449336bafb3111882f59369fa5ef03288cd5847a3b8b86f62739a',
    (0x15a6f0,0x15a78c): '87c529989345bb1df12a2e70a1d1a6b73d93a8b84a438f8f32a577dd37fec763',
    (0x159140,0x159150): 'a8500101af091d6d9ce554234d8be36aee2ae21748df7d30150f67fc7ec6c0f8',
    (0x120fa0,0x120fe0): '29204e947fbf9fa9b69420ae1cae478318746c39e977062cd82bd5943da726a5',
    (0x120eb0,0x120f6c): '6439d105d9c5b0c6f64ec233e0d91cae9990c0f234df6127b82b0f8d219bb376',
    (0x118230,0x118320): 'fd8d38b9afa48075325def7aa1b70178fbb141c758539d0220addeee4e804cce',
    (0x102650,0x1029c4): 'ce64aef481b0dc8fb22939b74dbadc708dbd4cf8a338a625fb390361d5ee421d',
    (0x105a30,0x105b78): '98ad977066058967df8a5adeaa753e6a98f451da7d58775c94e688dda5b2adf1',
    (0x105940,0x105a24): '8905e3dfa8687e32bd4b3a8d42d3ab74d5c64fa7b529b3037171a13809b5fb3f',
    (0x1131c0,0x113344): '8a47682b3758bc684b4f47963661c23808e1ee408ed765a20514c440668e00cf',
    (0x153140,0x153160): '9d5eb316453da2fc6396c14e5ac2dc016c6c19d195a113d1ade57479a696009b',
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


def reject_performance_counter_reads(data):
    # Static input admission only. The native runtime has no COP0 counter bank.
    # If a later image consumes this instrumentation, audit/port that consumer
    # before retaining the write-only reset retirement.
    _,_,sections=elf_parts(data)
    for section in sections:
        if section[2]&6!=6:continue
        for offset in range(0,section[5]-3,4):
            word=struct.unpack_from('<I',data,section[4]+offset)[0]
            if word>>26==16 and (word>>11)&31 in (24,25) and (word>>21)&31 in (0,1,2):
                raise ValueError(f'COP0 performance consumer requires a native owner: {section[3]+offset:x}')


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


DISPLAY_STORES=(0x1142c0,0x1142d0,0x1142e0,0x1142ec,0x114328,
                0x114334,0x1144c0,0x1144d0,0x114520,0x11452c)


def display_adaptations(data):
    _,segments,_=elf_parts(data);lo,hi=0x114240,0x114544
    s=next(s for s in segments if s[0]==1 and s[2]<=lo<hi<=s[2]+s[4]);at=s[1]+lo-s[2]
    if hashlib.sha256(data[at:at+hi-lo]).hexdigest()!='3ac33477475d7c3e7057cf03c23bf12a2e7c6450ce1154706b25b20f7e589c51':
        raise ValueError('USA display selection source mismatch')
    # Keep all software buffer selection, movie records and helper calls.
    # Physical DISPFB/DISPLAY writes have no native presentation consumer.
    return ''.join(f'{pc:08x} {int.from_bytes(data[s[1]+pc-s[2]:s[1]+pc-s[2]+4],"little"):08x} 00000000\n' for pc in DISPLAY_STORES)


def startup_clear_adaptations(data):
    _,segments,_=elf_parts(data);lo,hi=0x100694,0x100744
    s=next(s for s in segments if s[0]==1 and s[2]<=lo<hi<=s[2]+s[4]);at=s[1]+lo-s[2]
    if hashlib.sha256(data[at:at+hi-lo]).hexdigest()!='ceee7fc9a1cf2420fc202b6054eea642b60e95a3c423ecfbb0f10bef0d07f75b':
        raise ValueError('USA startup graphics-memory clear source mismatch')
    # This boot-only loop zeroes all 4 MiB of physical graphics memory in
    # sixteen slices. Native rendering has no such memory. Retire only this
    # upload producer; preserve the zeroed EE buffer, descriptor construction,
    # all sixteen frame waits, and the buffer's free. Real image uploads reject
    # until the port has installed an asset owner.
    return '00100708 0c048afe 00000000\n'


def composition_adaptations(data):
    _,segments,_=elf_parts(data)
    for lo,hi,expected in (
        (0x114550,0x114dd0,'b8cb45e4aaac55cca40d406b822f252e25d615d95e763cc0187a50f7218a44d0'),
        (0x1166e0,0x117120,'8c3492d778af719aad095216588ce3d3bbcb231c533ddb8b0919664c4e0ba76c'),
        (0x1178f0,0x117b00,'e62c5ed79538f787f4adfd44ac28638502753bbe4a51e7a8d66af4c1c021b557')):
        s=next(s for s in segments if s[0]==1 and s[2]<=lo<hi<=s[2]+s[4]);at=s[1]+lo-s[2]
        if hashlib.sha256(data[at:at+hi-lo]).hexdigest()!=expected:raise ValueError('USA source-target composition signature mismatch')
    # Only the admitted ordinary path. Other branches retain their transports
    # and reject at the native entry until a separate native owner is added.
    out=[]
    for pc in (0x1145f4,0x114630,0x114710,0x114a74,0x114770,0x1147a8,0x114ab4,0x114ac0):
        s=next(s for s in segments if s[0]==1 and s[2]<=pc<pc+4<=s[2]+s[4]);at=s[1]+pc-s[2]
        out.append(f'{pc:08x} {int.from_bytes(data[at:at+4],"little"):08x} 00000000\n')
    return ''.join(out)


def composition_sites():
    return ''.join(f'native 0x{pc:x}\n' for pc in (0x114550,0x1145bc,0x1145f8,0x114634,0x114714,0x114a78,0x114da0,0x114774,0x1147ac))


def debug_font_adaptations(data):
    bindings(data)
    # The native instruction at the original enqueue's delay slot rejects
    # every writer except a specifically owned state-only font publication.
    return '00120f48 0c044c70 00000000\n'


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
    composition_adaptations(data)
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
    return 'native 0x1003c4\nnative 0x11fb50\nnative 0x11f62c\nnative 0x1131c0\n'+asset_sites()+composition_sites()+'native 0x120f4c\nnative 0x120f58\n'


def asset_sites():
    return ''.join(f'native 0x{pc:x}\n' for pc in (0x102650,0x102998,0x105a30,0x105b58,0x105940,0x105a10,0x118230))


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
    if name=='mod002':
        _,segments,_=elf_parts(data);lo,hi=0x3617f0,0x362764
        s=next(s for s in segments if s[0]==1 and s[2]<=lo<hi<=s[2]+s[4]);at=s[1]+lo-s[2]
        if hashlib.sha256(data[at:at+hi-lo]).hexdigest()!='d6e51ee0140ca3301f0f70481b2db4caa9f92b89a4da2b0209e68e220f935aed':
            raise ValueError('USA menu frame-loop source mismatch')
        lo,hi=0x360dd0,0x360f44
        s=next(s for s in segments if s[0]==1 and s[2]<=lo<hi<=s[2]+s[4]);at=s[1]+lo-s[2]
        if hashlib.sha256(data[at:at+hi-lo]).hexdigest()!='5c82211252cab0d3265c33966ab6bb1ccb433758cf6b340f51632fa109c41d4c':
            raise ValueError('USA menu graphics-memory clear source mismatch')
        for lo,hi,expected in (
            (0x372180,0x3721b4,'00a7de71ad33ba14ff16757ec861f55b56b4e4d0d166aae90d2c7a984e7f652a'),
            (0x372060,0x372180,'099e335567888b5edfa7c52a63078a02481904bf55ba859ae7fdf3a669e063a9'),
            (0x3721c0,0x37293c,'38b3ca7ed66c5b7be98e324863ac85ec73a73df5ed097d698710d313b8e4acbe'),
            (0x372040,0x37205c,'e6e81f90b768228012c77b0ff794a4068ceb0187abc59dc45364159a238dd8ae')):
            s=next(s for s in segments if s[0]==1 and s[2]<=lo<hi<=s[2]+s[4]);at=s[1]+lo-s[2]
            if hashlib.sha256(data[at:at+hi-lo]).hexdigest()!=expected:raise ValueError('USA native screen rectangle owner mismatch')
        # The native frame clock already owns elapsed frames. Retire only the
        # old HBLANK mode and per-frame counter reset (both in delay slots).
        # Keep every game record, call, wait and loop/checkpoint unchanged.
        # The menu's fill initializes only physical graphics memory. Preserve
        # allocation/failure, byte fill, descriptor construction and free;
        # retire only its upload producer, never the generic upload consumer.
        return '0036224c ac220010 00000000\n00362760 ac200000 00000000\n00360ef4 0c048afe 00000000\n003721ac 03e00008 08087268\n'
    if name!='mod001':return ''
    _,segments,_=elf_parts(data)
    for lo,hi,expected in (
        (0x3140e0,0x314120,'5b1ac88e48d62e6ae747d08f015c4b4e5a80c15e7ae4e198d6b8386eaf81ccd9'),
        (0x2e74f0,0x2e75bc,'249a16a05dfd11f8b9ee28c571dfc511db7d2b6e9d5b35207a992be32ee2c767'),
        (0x2e7e90,0x2e8678,'6a2e9aae85037a750a67937b87fd1fb9ec416ef0825fd3851a07ea449086d6ab'),
        (0x2ec9c0,0x2eca00,'0d77f0022583634e23d83ec8a1d7d616ec9d7fccfa546e66300632bec4b85191'),
        (0x2ec260,0x2ec320,'88082bea97d678aceef5d9df4523e46e7bad90bbbc4636e756bbee1f4de767a0'),
        (0x2ec320,0x2ec390,'63a04bb5444ce69a065df6e7897489249ef5df38f873e9ed6efe301e5f55ff9d'),
        (0x2ec390,0x2ec610,'bd63f498af7cda748d7235280cc32a239373f3cfe30b31b1a06d61a48a801fd5'),
        (0x2efe40,0x2f0010,'9c34d28ab52ec30569a1751d974d75e372989eb6939c2241e5deb114f34e8f5f'),
        (0x2eb3a0,0x2ebbe0,'90c38c4355f5e36e4f66a18431d3c03d960977ad76d1bc8f7c20ed2a3ea22ef6'),
        (0x2bbfc0,0x2bc1dc,'fa2e265514606b372d7420091f9af78c0661ef40447ddd477411dc486df7a307'),
        (0x2bc1e0,0x2bc31c,'3599cf1c9de1214629ffc9880dd26d1ff97dd85f8c5234de885e6aa6bb50d6db'),
        (0x2bc330,0x2bc3f4,'e423f073b74534f66f1a75b4bb88abe49db71199224aa7410cdbcd9deced1cf8'),
        (0x2ba600,0x2ba6d0,'d5c1383fece3d5aef471e7077549bb4804ee9ddcdc40fca4fc72c5edc3f9e707'),
        (0x1f02a0,0x1f0350,'1438189477cd535e605306838a3b166c742f6c4238d9670fb65af21aab717323'),
        (0x2be600,0x2be940,'f71f6f8ce0b2662cee044735531ee8e0dfaa59cb116a0c28265d03c44b909dd6'),
        (0x2f0de0,0x2f0e70,'1898503143ca6ccba1d99bc209d3a7fd3c9e9a0babc361b702fb1267df6eb301'),
        (0x29af80,0x29b2c4,'ef3a9f1a6cae966ab8a687451e72ac175d88b8e9883d09449949ca5c3389d0bf'),
        (0x29c990,0x29cb18,'361b601c33bd3f57fd190aef95f4716cd08b5bbdf401f852b6e80eb96933e9e2'),
        (0x2bef20,0x2bf0dc,'2562b94cada2f70f71933a401cc6eacc7c1eb0a71f60dafc6b7179d58fdb0fb7'),
        (0x248fc0,0x24926c,'c80605a259562aba190d4009dda4f3f913e461937e47c6e4dca959f8f81624cc'),
        (0x213fd0,0x2140f4,'929eb425f5d11c4d38fd2c0d2c18b8bad91718a8fe3845ba40df7e08a9392768'),
        (0x358490,0x358690,'9a1c658a34ce5474bcf366b3d05296b4eb6de58357684808a64cdcbe12b91b53'),
        (0x216e40,0x216f4c,'422ab7a80ab28b304ccdcd915c85f68fd5077dece21a302982f240d80845ccc8'),
        (0x213b50,0x213e9c,'1fe7bed2ee6202570c935cf1a286364c7164bc80ecd2eea8da2448bd923f046b'),
        (0x216960,0x216a60,'e14b399a6c5f8f7ed8ebd20d7f86d40e384730df022339458a1b94418c582bd4'),
        (0x208cd0,0x2091d8,'f3ee4da5dccf5e996d4fc172958ddc4e11df5e35b6cfe27daf11635b6c021369'),
        (0x207d70,0x2081c8,'555bb9c0bac90acded56c0dbb87467673cdd6f05d9c7c3c76ee3a6beb09eccef'),
        (0x208360,0x208470,'442f78da91dbd9481266249b0cefb0ea5f0f106808612e7f465689586b6e3f2e'),
        (0x20f440,0x20f938,'647886c3f393394904797808cb6d91bee33134f84c03343dec04d8e5a64b7173'),
        (0x207cc0,0x207d70,'1144a7e285dc63e4929e2b6b4dab7465e7307fcf43882e1ca4525de6670aa983'),
        (0x2205c0,0x220d14,'7431010872a544c9aff4e53de3a7afb8c8855b1a15f8576d847709649c272798'),
        (0x29e310,0x29e378,'0c5bbe60fbde143fea0d349ffc2d1aa13dc1854778712938b393544af0e39a9c'),
        (0x29e420,0x29e4a4,'62fe296e3e7fe5e2f6fd1601f6576e4f114439e6127602b6890027f427aebd1e'),
        (0x221030,0x2210b8,'001be789bc13df36bb7c4ba82f73c495d245d6b51233fff5ec5c07fe4aa603b6'),
        (0x221260,0x22177c,'8707e84a57715bd8d264d96365a7daf8a2bd2a0b5e69ca30cce94622975f5a23'),
        (0x29d730,0x29d7ac,'ab15a11b1d355d334466795d7bfda2e9f1880b3c29be10b5eb9fccdea5e17104'),
        (0x29d6b0,0x29d72c,'2c56f08bc47c7eb3d9da06553666dc0823551eff916d28f9fee65ff6f8673787'),
        (0x29df60,0x29e06c,'28fcd50a9725807ccabaf7ab63330b908621fa8ab98e44a634f1d777cce36dc3'),
        (0x2088d0,0x208900,'3c660370520146baad6bedd158b3039156ae25383c9800729cb3dbc2905f9978'),
        (0x208870,0x2088c0,'fbb3c497f42aeff52417da1a45846ac4bf98fdc8672b658f606d0bf1880408d4'),
        (0x21c9a0,0x21ca50,'70968fdcfc854775bb720d5a3059caae2c7bd6f5b7ecfda8be9750aab68f0ef6'),
        (0x2cb470,0x2cb4a0,'d75aad57b1cf26127f35bdf8a0eec8259d79c6b1cb8cbbabe269467dac4546ef'),
        (0x2cb400,0x2cb450,'d5a82a827e14f659ff760bf50123311b26c433680f853e9caa6ba6ccbccaaada'),
        (0x2cb640,0x2cb8d0,'9707a3c9db31f1d7e805ec29ccb2ce31bd6922b8dc52da460a8ce41b381919da'),
        (0x2f36f0,0x2f3724,'f6ab42d7b64064e3139b5273ddb2c37c95ee51afbf05937b631d50a7fa880445'),
        (0x2f37b0,0x2f38f0,'6161fececd96ae665ac117bebfbe8a5912ddbaf0811a3aea530c5e7ae77c30d7'),
        (0x2ea9a0,0x2eaa10,'51d9d49a527ee0894614cd805a3bde2fff0706aeeb84385ff002ff783750b4be'),
        (0x2210c0,0x221260,'97a2782e507e8accd9be2dfc3a3da1e16d8c79fdb9783f48b52151f921c40ebd'),
        (0x29aeb0,0x29af78,'7e822b8520d71ee253400f4e8565e933bb9a9495330e461385d2fe04ef673f69'),
        (0x29b480,0x29b520,'195dd1778735679111fe1a0415b42159a229bc6999f4b53594a49cb9ccb55b99'),
        (0x249720,0x24a438,'10c8d5e49e540decaa624ca4ea5f9e5aeee7cc23f9acf910d53cbe623ad1484e'),
        (0x202150,0x202200,'2056326076193b416411c06ca8c0b005e58038dd2b237625a6242ce3d0b661da'),
        (0x29e230,0x29e310,'62230c5e1d41b56528bb4de238ce9919fef2df3c7043e5afa91f54b1b1dce634'),
        (0x29cc30,0x29cf58,'fad60754970a95b73f1a675b374267b9c468cfac43d0388aafe45e8c83e38a48'),
        (0x29cb20,0x29cc30,'07073874d74c68aea7c1f838455b4d30b523189ea9ea03406dc4d6cce8a78e16'),
        (0x2b8f90,0x2b8fb4,'28aa89f14be29a42a98079198bcd1cc6e83ed61b0dd12791fcd8f41eabdc1a1d'),
        (0x314200,0x314600,'1143517174b62b3091763659810036f9cece68582ce77724d655249c30487f79'),
        (0x29e0b0,0x29e1c4,'4badfc2bd23ca38b95965aa45332f82c6efd9655b77f0d7b9344126467e8745d'),
        (0x312280,0x313100,'a834d66f34036f87f4061c9e5c2d623b29ef77f6fd6394b765333d0f72c45617'),
        (0x314138,0x31413c,'e7f5662273401b756d9f64b866d1a7df7e95c248c34f9b0cad011eb84757f186'),
        (0x217550,0x2175c0,'bf29f73fdf9756c87ee0b7b11aaccb586cb29e0fb0653934452e96586c48b694'),
        (0x215870,0x2159c0,'c3cd1b33f97be33016d939dcf099aa81f1dddc53735f661478db065e35ed1c47'),
        (0x208900,0x208aac,'18ce58f2b4fb80da5a27b84951044a913c3ad676c01eb8456f4ab893de24f9ee'),
        (0x2092f0,0x20975c,'2060db8cd95539c247455d89bf3688dddfcc7905556baaa94a3afaa96583ab59'),
        (0x2097c0,0x209818,'8d94153af0cc888db971b38ac30c105f8201685dd02ddb8c86578c1a65767f22'),
        (0x2494c0,0x249720,'c017bf6d329e88332552da7c1f5949a763f4900862bc056f18fbd56b1a9848df'),
        (0x249270,0x249390,'5ddcd40fb0819990a6319eb8774b68d84ced2390068859deb276a6acf162c1ce'),
        (0x211da0,0x211dc0,'d7d4b93075083283a98e463e5ae9b850da09436519ccbb38b52e01deeae832ac'),
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
            # Keep the canonical game texture-manager record in EE memory.
            # No copy is made, no physical job is queued, and no completion
            # condition is served. Its guarded scene frame owns this lifetime.
            '00221300 0c0a75cc 00000000\n'
            '00221554 4100fffa 00000000\n'
            '00221568 0c04df7e 00000000\n'
            '002216fc 0c0a75ac 00000000\n'
            '0022172c 4100fffa 00000000\n'
            '00221740 0c04df7e 00000000\n'
            # Native camera selection snapshots game matrices before any
            # camera VIF job exists. Never acknowledge a generic DMA consumer.
            '002088ec 0c0807c8 00000000\n'
            # Native brightness submission owns its copied inputs before the
            # original software completion runs; no physical job is enqueued.
            '002cb48c 0c0807c8 0c087268\n'
            '002f3710 0c0bbce4 0c087268\n'
            '002ec9e0 0c0bbce4 0c087268\n'
            # A native decoder owns the prepared media buffer. Preserve its
            # original CPU reset/tag construction; never create an IPU DMA job.
            '002bc120 3c011001 080af074\n'
            '002bc124 ac20b420 00000000\n'
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
            '0029e160 a0820050 00000000\n'
            '0029e16c 0c0807c8 00000000\n'
            # Write-only profiling reset, never read by the generated images.
            # The original object-update owner and every callback remain.
            '002bef34 4080c000 00000000\n'
            '002bef38 4080c004 00000000\n'
            '002bef3c 4080c005 00000000\n'
            '002bef40 4080c006 00000000\n'
            '002bef44 4080c007 00000000\n'
            '00208a90 0c0807c8 00000000\n'
            # Publish the immutable offline asset at its CPU producer, then
            # free the unused descriptor through the original game heap call.
            # Do not submit a DMA job or fabricate its completion.
            '0029cd7c 00a0302d 0220202d\n'
            '0029cd80 0c044c70 0c0402cc\n'
            '0029cf14 00a0302d 0260202d\n'
            '0029cf18 0c044c70 0c0402cc\n'
            '0029cc00 00a0302d 0220202d\n'
            '0029cc04 0c044c70 0c0402cc\n'
            '002c3cb4 8c220000 00000000\n'
            '002c3cb8 10000015 080b0f48\n')


def overlay_native_sites(name,data):
    if name=='mod002':
        _,segments,_=elf_parts(data);lo,hi=0x3622e8,0x362338
        s=next(s for s in segments if s[0]==1 and s[2]<=lo<hi<=s[2]+s[4]);at=s[1]+lo-s[2]
        if hashlib.sha256(data[at:at+hi-lo]).hexdigest()!='6d87622143495b5e67a67c1a1655a2eef28d48c7c1d78309ea18a3ad3e6f45ed':raise ValueError('USA debug-font publication caller mismatch')
        overlay_adaptations(name,data)
        return 'native 0x362328\nnative 0x3721a4\nnative 0x3721b0\n'
    if name!='mod001':return ''
    overlay_adaptations(name,data)
    return ''.join(f'native 0x{pc:x}\n' for pc in (
        0x29e1a0,0x2c3cb4,0x29cc30,0x29cd7c,0x29cf14,0x29cf30,0x29cc00,0x29e160,
        0x208a94,0x29af80,0x29b2a8,0x29caf8,0x2205c0,0x220ce0,0x29e43c,0x29e444,
        0x22106c,0x221294,0x221308,0x221540,0x221704,0x221718,0x221748,0x2088f0,
        0x221698,0x22169c,0x21c9dc,0x21c9e0,0x21c9e8,0x21c9ec,0x21ca1c,0x21ca20,0x21ca24,0x21ca28,0x2cb490,0x2f3714,0x2ea9a0,
        0x2e74f0,0x2e7520,0x2e7550,0x2e757c,0x2e75a0,0x2f0de0,0x2e83e4,0x2ec9e4,0x2ec308,0x2ec320,
        0x2efe40,0x2effe8,0x2eb3a0,0x2eb5c4,0x2ebac4,0x2ebb0c,0x2ebb58,0x2ebb9c,0x2ebba8,0x2bbfc0,0x2bc1d0,0x2bc1e0,0x2ba670))
