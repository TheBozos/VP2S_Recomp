"""USA-only guards for retiring SDK firmware syscall/alarm installation.

No instructions or load bytes are changed. These are port-owned native binding
declarations, deliberately separate from general SetSyscall support.
"""
import hashlib
import struct
from vp2_bozo_prepare import elf_parts
LEAFLESS_QUERY_SITES=(*range(0x1f6f94,0x1f6fa8,4),0x1f7098,0x1f709c,0x1f70a0,0x1f70a8,0x1f70ac,0x1f70b4,
    *range(0x1f70f0,0x1f7178,4),0x1f717c,*range(0x1f7194,0x1f71a8,4),*range(0x1f72cc,0x1f72e0,4),*range(0x1f7334,0x1f7348,4))
FULL_SINE_SITES=(0x2143f0,0x2143f4,0x2143f8,0x214404,0x214408,0x21440c,0x214410,0x214414,0x214418,0x21443c,0x214460,
    0x214468,0x21446c,0x214470,0x21447c,0x214480,0x214484,0x214488,0x21448c,0x214490,0x214494,0x214498,0x21449c,0x2144a0,0x2144a4,0x2144a8,0x2144ac)
FULL_COSINE_SITES=(0x214220,0x214224,0x214228,0x21422c,0x214238,0x21423c,0x214240,0x214244,0x214248,0x21424c,0x214250,0x214274,
    0x214298,0x2142a0,0x2142a4,0x2142a8,0x2142b4,0x2142b8,0x2142bc,0x2142c0,0x2142c4,0x2142c8,0x2142cc,0x2142d0,0x2142d4,0x2142d8,0x2142dc,0x2142e0,0x2142e4)
EULER_QUATERNION_SITES=(0x216390,0x216398,0x21639c,0x2163a4,0x2163f4,0x2163fc,0x216400,0x216408,
    0x216458,0x216460,0x216464,0x21646c,0x2166d8,0x2166e0,0x2166e4,0x2166e8)
POSITIVE_EULER_SITES=(0x215a8c,0x215a94,0x215a98,0x215aa0,0x215af0,0x215af8,0x215afc,0x215b04,
    0x215b54,0x215b5c,0x215b60,0x215b68,0x215dd8,0x215de0,0x215de4,0x215dec)
DOF_MATRIX_SITES=(*range(0x2d7c04,0x2d7c74,4),*range(0x2d7c80,0x2d7cf0,4),*range(0x2d7cfc,0x2d7d6c,4))
INVERSE_PIVOT_SITES=(0x216f54,0x216f58,0x216f5c,0x216f60,0x216f64,0x216f68,0x216f6c,0x216f70,0x216f74,0x216f78,
    0x216f80,0x216f8c,0x216f98,0x216fa4,0x216fb0,0x216fb4,0x216fb8,0x216fbc,0x216fc0,0x216fd0,0x216fd4,0x216fd8,
    0x216fdc,0x216fe0,0x216fe8,0x216ff8,0x216ffc,0x217000,0x217004,0x217008,0x217010,0x217020,0x217024,0x21702c,0x217030)
OBJECT_UPDATE_POINT_SITES=(*range(0x227d78,0x227da0,4),*range(0x227de8,0x227e10,4))
OBJECT_QUERY_POINT_SITES=(*range(0x227f08,0x227f30,4),)
OBJECT_LINEAR_SITES=(*range(0x229164,0x229184,4),*range(0x229228,0x229244,4),0x229248,*range(0x2292f8,0x229318,4))
OBJECT_CHANGE_SITES=(*range(0x2295d4,0x2295f4,4),*range(0x2295fc,0x229618,4),0x22961c,*range(0x229628,0x229648,4))
PARTICLE_CALLBACK_TRANSFORM_SITES=(*range(0x410660,0x410688,4),*range(0x4106b0,0x4106d0,4),*range(0x4106f8,0x410718,4))
HERMITE_VECTOR_SITES=(*range(0x2a68d4,0x2a68f8,4),0x2a68fc)
SQUAD_SITES=(0x214c40,*range(0x214c98,0x214ca4,4),*range(0x214cbc,0x214cc8,4),
    *range(0x214cdc,0x214cec,4),*range(0x214db8,0x214dd4,4),0x214dd8,
    *range(0x214df0,0x214e04,4),*range(0x214e08,0x214e20,4))
PARTICLE_PUBLICATION_SITES=(0x22eae0,0x22ec9c,0x22eca0,0x22ee88,0x22ee94,0x22ee9c,0x22eea4,0x22eebc,0x22f0a0,
    0x22efd8,0x22efdc,0x22efe0,0x22efe4,0x22efe8,0x22efec,0x22eff0,0x22eff4,0x22eff8,0x22effc,0x22f004,0x22f008,0x22f010,0x22f014,0x22f018)

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
    (0x15bd70,0x15bda8): '745c998c9dd5a82d9ed8cea079133db021b52b550e89449d5e1b894b71a7a9d5',
    (0x1068e0,0x106a2c): 'b30262707bb8bd7ca27139232189b1698780b94cd13fe048685c3758c4708942',
    (0x15a1e0,0x15a200): '3d5516bcb133aad0302be1420e75fd0d283d1ad45aed193efeaef36368eb110f',
    (0x152060,0x152080): '82feb318e1227eae8460ec6bb489653ef560aab596c9ec7ef9a04b74df95c62a',
    (0x15ba00,0x15babc): '01c9487bea078d22ae8dea8d82ce3f48ce0d593fe75191f3598e7e805e987002',
    (0x15bca0,0x15bd6c): '41dc9e8abb846d8e4cd5a85b4b1cfbd7029c50ad0ff84902148c7376bcf7a8d5',
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
    (0x105b80,0x105f58): '80bf0eced254405b1ad468a0bee04304335f1f9309ac2cd030e752c4fa7f4a9a',
    (0x1060a0,0x10626c): 'ee2537548bb53d4d4fbb587300a0afab6c94f34f32b078eb7ddd9309d3f59c1e',
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
    lo,hi=0x134b60,0x134d18
    s=next(s for s in segments if s[0]==1 and s[2]<=lo<hi<=s[2]+s[4]);at=s[1]+lo-s[2]
    if hashlib.sha256(data[at:at+hi-lo]).hexdigest()!='de60c4b86c3704fc5eeafe80a737afa57dd43ddce1439476fc7d6ba49aa4f72b':
        raise ValueError('USA bounded string comparison signature mismatch')
    entries.append(dict(Name='bounded_string_compare_00134b60',Start=hex(lo),End=hex(hi)))
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
    if name=='mod003':
        _,segments,_=elf_parts(data)
        for lo,hi,expected in ((0x3749f0,0x374a04,'a452358fd505e54aa6c7b852ca05fe160eeddde37d458dd1f515ee4591659823'),
                               (0x3e5bc0,0x3e5d34,'91249dbfeb8a6b82139f1ccc84edcbf38621ad5262ef54652ac17ae0969e8b31'),
                               (0x4126a0,0x412868,'ee7d09a1e0ac086dfb09b220417edf8f88ae9f9ba63092e747dcaca12634b7b3'),
                               (0x446520,0x446b8c,'c8c6018c966aec266f7ece12732d59dc2d5d132e3fc71fd4361046033ee154b1'),
                               (0x408e90,0x4095d8,'65e31072aaf7deec4ca8c5bde663e26b8e000e79f634d8a321bd254404d7b0f0'),
                               (0x40a8b0,0x40b07c,'cd09fe7920018dacc97ec1192c1ee41b355a7e3afdb5d025f439d0a77c1676c3'),
                               (0x420ca0,0x4210c4,'b92572ee6121f449ec4445f9874096b68fbda868dee7bcbdf08a28c7d1664037'),
                               (0x410640,0x4107c4,'ff6bd98076903b5882e141046754b6f8e303f5b1863a79a584fc40c8b6e668e8'),
                               (0x377d20,0x377e20,'399b8a22edce4516229aea8a3721ede1f3316ac126f13c89c13ef3c40d49507b'),
                               (0x3800f0,0x380178,'da61801422de732fba849adb7f586150c05c1902c38327e985a4ee7e1c6f34c3'),
                               (0x3805f0,0x380738,'f99bce02f1bbb8aadad57eb274856e29eaec38cbbb9783432c141e6ce401b093'),
                               (0x3807d0,0x380918,'96443dbe9708b70866512175223295131ef04a96f7789cf2829b3b232f1faa68'),
                               (0x39f300,0x39f3ac,'1e1eec457a4706387c92ec3864026df1a64657a562f78358a56a945c8cdd0cde'),
                               (0x39f0d0,0x39f2f4,'fa1b55f8495a559439e1073ad09d23f5c777c8850ae11087decf613b6fabed24'),
                               (0x439180,0x439d88,'a22f24d7862e9f68dc5a18f4537128e5a23235fb7544dc739b2428c405288b76'),
                               (0x3cdb60,0x3ce85c,'dc02dbb7674750652c798f2b1e34c4bfb5c78785087ed7760ce9465c812705cb'),
                               (0x3ef010,0x3ef230,'8ece0548612d0b75ad1d0830c489b26f0ea694f45d25d1120f2dd34fca85d9e7'),
                               (0x3e7e94,0x3e80f0,'9d7958bae8751020c5857db7a226947e203b17a7434cd0b937267dd3da47bed2'),
                               (0x3d7708,0x3d77cc,'435926535d852e22f0b20b9027f65fd6674acc7a63b6aaedb31e59c63a1cd1c8'),
                               (0x3d5670,0x3d57ec,'250a2389769015fbda23859f5082b6f69031a706af85f263b765f34130611ecc'),
                               (0x378908,0x3789c8,'25cee96dbb982fe7f31c806c09286d7d6d1630101851998b42a1431e9cf4fd17'),
                               (0x381fb8,0x38204c,'90b99ee0965ca566fb0e0a98e84ced53026a0170815095589932282d774eb388'),
                               (0x37fa30,0x37fb44,'295d39997772ec085aa90c9fa153dd4ca80ae48d6d4f4f4e71b04c4aa2948773'),
                               (0x37f95c,0x37f9d8,'b00b064ef0abadaff9d7316fa55bb263dcde9b312b2bed49de78769aac199804'),
                               (0x43d280,0x43d35c,'c2e7f59ac40b98663131b006afdc7423baa1db4f7d85b29c9e790500fcb5e4b1'),
                               (0x3c5d08,0x3c5dc8,'000f0a47d3b89e66148c4d84eba1523b103743a3077c4984d2037d2b1264345c'),
                               (0x379228,0x37928c,'633fb2f7985a6967967d045de01711e611a139f1dcbd627455c7f060aea9273d'),
                               (0x410df0,0x410e50,'01f0ab9d247a3789f43323e8ee961cf30f4372e3c0d152471e6b32e89dbc32ef'),
                               (0x37b500,0x37b578,'7e6a36380abfc775a0388113d7bf0c9eca5fecc602e98a0ea43e0b071d0b47b4'),
                               (0x3d93dc,0x3d9400,'f8fa0f2fe2b3f9889f704dfc20ca3806981c87fd37c7904b13e77c6cc7afc39c'),
                               (0x3d9230,0x3d925c,'60fa72c319ee92c05082ae3e1496caba1b37138f89c8e22411be03477740c5d8'),
                               (0x3d95ac,0x3d95b8,'e53ce69864c095f0410aa0422eb78527d3c44442b5a5f02312e8353d3e0cc3f7'),
                               (0x3d3620,0x3d3660,'fb483410366f6db0a0a09b4d1b6a178331a6e6be6ab08fb8083b3ecf01a2e150'),
                               (0x3d3660,0x3d44e4,'c7d0d58342fcb2eb8e696c6afc145c73ad44f487a94acf685078c1d7a5eec94a'),
                               (0x3d49b0,0x3d4af0,'3ee241a66da54d7847e111212bc57e162a559141b1093ace33ce0a314948ed10'),
                               (0x4479e0,0x447f50,'3dff331f2b436cc1b41965930174f4b18a70a204e69488ae0263232a22827f3b'),
                               (0x447f50,0x4480a8,'fd4a4ee0d65621b54a09bfe7315461f498a1b0de0167ad1450531b0c967d4093')):
            s=next(s for s in segments if s[0]==1 and s[2]<=lo<hi<=s[2]+s[4]);at=s[1]+lo-s[2]
            if hashlib.sha256(data[at:at+hi-lo]).hexdigest()!=expected:raise ValueError('USA native field initialization source mismatch')
        # Each initializer has returned before this delay. It polls only timer0,
        # not a job/status result. Keep all36 calls, one visibility flush per
        # record, and the original UI predicate/pump; retire only the time spin.
        # Native frame targets clear their own colour/depth/stencil. This
        # separate physical fill has no native graphics-memory destination.
        # Keep the original CPU allocation/fill/descriptors/cache reset/free.
        # Field pacing still uses the real ported frame-event API below and
        # in the update loop. Its old timer0 HBLANK setup/reset owns no native
        # counter; neither write produces a result or completion to replace.
        return ('003d9238 ac220010 00000000\n'
                # Animation precaching keeps its original 128-sample bound,
                # model callbacks and completion. Its raster-time yield has
                # no native clock or hardware value to substitute.
                '00420ed8 0c04343c 00000000\n'
                '00420ee0 3c010001 10000013\n'
                '00420ee4 24060002 00000000\n'
                '003d95b0 ac200000 00000000\n'
                '003d5694 0c04896e 00000000\n'
                '003d5794 0c048afe 00000000\n'
                '003d57a0 0c04896e 00000000\n'
                '003d7718 3c011000 00000000\n'
                '003d771c 8c360000 00000000\n'
                '003d7740 3c011000 00000000\n'
                '003d7744 8c220000 00000000\n'
                '003d7748 00561023 00000000\n'
                '003d774c 2c41003f 00000000\n'
                '003d7750 10200005 00000000\n'
                '003d7760 1000fff7 00000000\n'
                '003d3648 dfbf0000 0c0f4d38\n'
                '003d364c 03e00008 00000000\n'
                '003d3650 27bd0010 dfbf0000\n'
                '003d3654 00000000 03e00008\n'
                '003d3658 00000000 27bd0010\n'
                '00447d5c 0c044c70 0c111e78\n'
                '00447a44 0c044c70 0c111e78\n')
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
            (0x373eb0,0x373f10,'133807cdfbc46afc446b142953dd60a1af23641b22a79bed4db05677cfae6492'),
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
    for lo,hi,expected in ((0x12b4a8+0x1f0,0x12b4a8+0x250,'b79cf2382262fa4251a0e08c8d0b7eb05f051c8a77915bf5863c78d7d6acd784'),
                           (0x12bbc8,0x12bc98,'5082777297b0b5b697227fc4ac15634e039e8a9c264cab5c4d0dc36da6b7096c'),
                           (0x12be08,0x12be48,'8bb8d21ec22cf7d97121ba65944b1a398b87bb09879e3b1cfb74f46550d28342'),
                           (0x12b4a8+0xd0,0x12b4a8+0x188,'de5d48299b40b3a851768c7895cbdb4449ecc08c3a585ca79a9d03d973a1c7e7'),
                           (0x12b4a8+0x7f0,0x12b4a8+0x838,'a07e1286b441ee14cfe8b3358b59ee04fed7789bb456988abf6ae5cd936dfa97'),
                           (0x12b4a8+0x8a0,0x12b4a8+0x960,'af7ecfd91505685a1f79a0870924e24d3b79f4c60d583d071090f16df8d183bd'),
                           (0x12b4a8+0x9f0,0x12b4a8+0xaa8,'1fe7818af2509cd6f7672bf368a5b2059d8f64e0c7b377ea8d9060ae3d790f2d'),
                           (0x12b4a8+0xc90,0x12b4a8+0xd60,'d3023b6ef8dceba14eedcde59ac05de06fccb110e6dbc8dff44b864eb221eb33')):
        if hashlib.sha256(data[lo:hi]).hexdigest()!=expected:raise ValueError('USA native math algorithm mismatch')
    for lo,hi,expected in (
        (0x252e30,0x253344,'0aa2f411a373a9d07357735229c7599e0d2f7ec1dbec3a6371e03c683b5bd448'),
        (0x1f6f40,0x1f739c,'64aa600e45325da65e5dbde01bf3d555bfcf683fd446955177b2c987b40246ee'),
        (0x1f8aa0,0x1f9128,'6b6f4904da15dc26a04bab0d45b242ed8137e59f6eb8f3a5da5fad53e50e1e91'),
        (0x209aa0,0x20a4bc,'475ada92e212bc095a32fa12a341da89bf20261019a6a7873e42a55a090ca0b6'),
        (0x1fc4b0,0x1fc5a8,'e08fecf06bb52a8de57369b0d98ca938222d1cde05391599aabcc7f54cfe2b75'),
        (0x1f9d40,0x1f9e94,'3bc735d6df7168a9ef3c92a7f7ebc8f70f7cca38dc8ac931dcd9474380ea4c14'),
        (0x2a44e0,0x2a4e20,'15f05b9197f1ccb591321e268c8d418765f1789e2e3e53c94f6d965e406cffe8'),
        (0x2a40e0,0x2a44dc,'e69c95cd19de0804fc985c8770ba247b2f895f71eef70f51862434cbc4876d6d'),
        # Collision-tree leaf tests (box SAT, sphere) called from 1f6f40.
        (0x1f6be0,0x1f6e10,'7ce5532ba5ba3597a21e26234650d7a9918f4479a355d888728c815be9d2728c'),
        (0x1f6e10,0x1f6ea8,'5da5a14092064e896a3dcf3ec7ab5488240a7f3bd634b284473d41a49834922d'),
        # Segment against the hit box and its scalar clip helper.
        (0x1f6740,0x1f6a38,'8f1a0059234b0a2a618cda8a7256f38c0d24c2c607e9084f830997710e4231cd'),
        (0x1f6670,0x1f6740,'92c9dc03014f3b4f3e397b26c1fbd1a159d01833f9ef6334666c662975fab1c8'),
        (0x246760,0x246ed8,'76eb263da293367e94e61afd20b6b6904f0e05bf5395ce7ed594dd6b1fd1aadf'),
        # Chain constraint owner 1fd110: placement, split, rotations, blend,
        # bone aim (1fe400 half constant) and the closing product.
        (0x1fd110,0x1fe9bc,'99519d39445c3ea2e6c3b2706ecc9c0fc8da51c813a4d4dbdfaf1080ecca3899'),
        # Capsule pair 1fc300 (virtual +94, called from 1fdc24 only).
        (0x1fc300,0x1fc4b0,'a9be5b8c031115865ed4655227693efe81c616e3e3655dabb7676b7c5689f0f5'),
        # Plane contact 1fbd60 (virtual +8c, called from 1fe208).
        (0x1fbd60,0x1fbe60,'10e4e985badcbb5ef1eb411c7d8e00be4218e387d2f5281d50f8c58faec8415b'),
        # Range test 1fbad0 (called from mod003 4466c0).
        (0x1fbad0,0x1fbc80,'576a329a563d8e6cfd35ef06d5d5b8d39425123b780e2c1394b8b9960f68e762'),
        # Matrix build 2178b0 (composition with the 216e40 result).
        (0x2178b0,0x2179e0,'b5461480644eaf81f1f1b135c124af2de68bbb0f2f49ef3ae9eaae4441e253cc'),
        # List box query 29f5e0 (called from 23e1a4).
        (0x29f5e0,0x29f760,'cc19d75139ab6ece72d006b5460694f94c19ae26f5da1af7eb4c003ce84ba757'),
        # Hit frame 23e0d0 (composition and row product after the 29f5e0 query).
        (0x23e0d0,0x23e498,'3244dcc3409474944a9641bf4e07a1f415ed75583db0480b586a2a81994c883f'),
        # Plane distance 214590 (called from 20d9d0 with the +7f0 sphere cache).
        (0x214590,0x2145c8,'22b9460cec064b34c979b925efe1cb77bf12fe1f6d1d830dcb63f1e64def09ee'),
        # Sphere sweep 20d9d0 (called from 23c0d0 with the +7f0 cache).
        (0x20d9d0,0x20e508,'2cad482dd0dc6c905742c07a95d7cfd468cafc555d8556f38c7285b2bd78e518'),
        # Animated packet UVs: the 2188d0 streaming owner, loader 219620, the
        # UV-scroll callback 29cfe0/descriptor method 29d0d0 and mesh cursors.
        (0x2188d0,0x219608,'efe18082ff8f33e4c91a09af3d3cc3e775706b30795ceaae059e845b6b23546e'),
        (0x219620,0x21a628,'ca46b6666b7ed13428280babbda428ef85fc0a64b86434b7d13c7af7a3b8e416'),
        (0x29cfe0,0x29d0c4,'2545c35e9aea1a70e55972cccfda34481d33407d8537c001c416c2c00501782d'),
        (0x29d0d0,0x29d130,'e7ed055bacffa988f4b68f098464d2b32b7f58560d021ea35fb149ec0712a534'),
        (0x25cb00,0x25cb28,'3be951a0f233c80eba60eb1fab6c26e2ec2b77f1641f26dadc695246e3265c42'),
        (0x25cb30,0x25cb58,'4e2e95c8fafa12529ef6ca4cd8bf45d14b2d8944295b644900ac88fd3278bd6f'),
        (0x25cc20,0x25cc80,'efcfa5bb374825bb407a92d0c84d371ab9cb10e65da0191371abfc0f9fa66080'),
        (0x25cc80,0x25ccf8,'a52053d7f6cadab9bf1ceccc9a5270538b8853ead415d8f4689a1583eee73b4e'),
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
        (0x2bc1e0,0x2bc330,'b74e741629d88970b322389e1c628bb3778a9ab95b017c6fec1858b6e30aec4d'),
        (0x2bc330,0x2bc3f4,'e423f073b74534f66f1a75b4bb88abe49db71199224aa7410cdbcd9deced1cf8'),
        (0x2ba600,0x2ba6d0,'d5c1383fece3d5aef471e7077549bb4804ee9ddcdc40fca4fc72c5edc3f9e707'),
        (0x1f02a0,0x1f0350,'1438189477cd535e605306838a3b166c742f6c4238d9670fb65af21aab717323'),
        (0x2be600,0x2be940,'f71f6f8ce0b2662cee044735531ee8e0dfaa59cb116a0c28265d03c44b909dd6'),
        (0x1ef6f8,0x1ef930,'6d8580c5bdb3b9f017e9635078ed56ef1395a7a37ac557743ddac5a3feefa731'),
        (0x1f06c8,0x1f071c,'39b01d58e7779a8c5a6ccd36aa27a905c4e7495d9b0168315d525fefb5a4d02e'),
        (0x1f08e8,0x1f090c,'1c9cc6c3c8bb8dc6eb023d0ca0929b18cc2b9ae616ecf086e0401c141cbf2863'),
        (0x1f0920,0x1f098c,'2c50080a1de3823d946c8004e448bf22721ff2242fdc891dc3e2f329a5424bc3'),
        (0x2ba4a0,0x2ba5f8,'c4d164a861b03b93c737ef97ccbc53fca761b3632d9970d4af87c6f045d3e31b'),
        (0x1f0510,0x1f0544,'ef4dd54fb7fa380f1adbfd8d8f16b0b68b29f0f05978fe6d09f38f2ea48a70db'),
        (0x1f4990,0x1f4ac0,'f970938499b6bb30dca26375898518134256bded2ddff122dd328a51b231af33'),
        (0x2ba3b0,0x2ba3d4,'b4a14ac0f25fde8d4e1ab075ff66b0faf283dc3f898a22471ecfd5cd5601cb47'),
        (0x313f00,0x313fa0,'15d9967557c6d7982e4ab466794dfb20e0b3e7760095b1d287db08c94fad44e9'),
        (0x1f0438,0x1f04b4,'eb3dd523d14ce5b8c9c58dc381746ca560d37fbb7bd0d09374a6307739e733e7'),
        (0x1efa30,0x1efaf8,'973d30e2a2a38cb0404f8684041462e72eacdeaa6f266cee39f15ea8d91e78b5'),
        (0x2b9b80,0x2b9c08,'fa5e9e0c93973781aaa5daec6f445c53d7575c6e98c7882b530d8cca28297237'),
        (0x1f1f70,0x1f2140,'29544734d5090bbe09bfc710e0a21ee576878effdddfaadd91f8e5d48949d5b0'),
        (0x1f2140,0x1f27e8,'54fb5c0baaa283b2b4d3843ac980b5081204a129bab98a68a467ad87aa628fee'),
        (0x1f27e8,0x1f4590,'2f62c1b0f7f2a33982fbb38b35fc7265d3e3a0906a2e1be6dea50c8371547252'),
        (0x1f4590,0x1f4970,'bc5c78d46ad3b0d07e5dcccdbe0ca495f17ea5e4bfc67f75a430010c4823df1e'),
        (0x2bdf48,0x2bdf80,'9feb197ce3886df366b0b311c38985db5e08beb7a7032c7d961849ef7e2f5ac6'),
        (0x356270,0x356280,'229a41bc6171d3d62f6e994da6cdfbf2c8c1bae0293fb1018a98b10f4ff91066'),
        (0x2b90a0,0x2b92e0,'3d7c76d046949235bda80828bb6f15dc6d789f37a5b7de87daba9952695024f4'),
        (0x2bd9b0,0x2bdc20,'609900e4b7175c36d7cb6ef3a4d770d4608a9f4e4f6d6e24c4f33e5b786b7cc5'),
        (0x2b9f00,0x2ba040,'0e98d8c1e6c6a061d9b2e93d130a945f54a866becb68fb53148413f7639cd976'),
        (0x2b9560,0x2b9b80,'ca9727ce56eb89f4b5f2906e75733d96774e687ca74d93c51ba97bc7872f87c8'),
        (0x2be020,0x2be600,'c338390f8bb0c2d68f423fac8d070b2104a9c155d91a0439accfa88531ff54c6'),
        (0x2bd400,0x2bd9b0,'fa1fa27b51c343d7d92f62b33d97f1283643909a05a5ae3e180a32c77e366bb6'),
        (0x2bea60,0x2beae0,'e65fba217935eec0156f4ff4238485e7cbce6af8fcee627ff778dcc5fe2c630c'),
        (0x1f04b8,0x1f0510,'c45c191234f978918398f5c195f0ae70b36c0caec1cf815ea9d31d6c5d6ecc08'),
        (0x239ce0,0x239e00,'6163fdc68fad046f3f7c7c0bb477c44e75f1b3a540a8515e386857e90cb8c1f5'),
        (0x312d50,0x312d98,'ba8d278c61a3f0210b69cf77912ae29e0122cea7addebaa98b15ab8d4d805d7c'),
        (0x312e10,0x312f38,'b8e931e093c5016b7ef512874a728c050b65b8eff7d00938147abf8993cb21ea'),
        (0x23c17c,0x23c18c,'3290340dd577659aedd5c3dae5aa65654209ed3d2eea6c083ea4d4e2561a7a3b'),
        (0x214300,0x2143d4,'330403e55d0411cbd0a00609cbb5672d93bd532b4a1a076337d2d7b9557d55d3'),
        (0x214210,0x2142f4,'c74a1b94b812fb97a7cf8fd776138335ca33ed3b500cdc09ea8db23234a2f5d8'),
        (0x2162c0,0x21672c,'b38b09a1be19e77828fe582ad416337a4e6d9673013fd4e8b7637fab54d7fbbe'),
        (0x2159c0,0x215e40,'e36261d96bff047183e5f65428408c24c5e2342cc66382efb2bbd6e6963a00d5'),
        (0x2d7a30,0x2d8ee8,'bec4803e6c91089551ff46e5dade2f4ab51e05c8789a4dfb1315c61e27f60b28'),
        (0x2a68a0,0x2a691c,'e8aed7e6dad0f2124495e8ea5c7137329ae4b35946053dd83054a3feb09317e9'),
        (0x2a6980,0x2a69f0,'7a6b363fbe7aa8325cb5dbe2afe50020d05f1998a5f0b1ae3f59f026fb37ca39'),
        (0x2dbde0,0x2dc6f4,'32cf1bcb8d09934d7c22d04ae23ef0a48c8dc55f659870f98d099cc3973b97fe'),
        (0x2dc850,0x2dc96c,'70077e83bc2a99b4e998d47c7985735f23ffabc8c4916b07ba2e47961f001a68'),
        (0x2dc700,0x2dc848,'dd166dd19de9b2d098d0b4c6dd0fb8f1fa97077e753c693759f15e371ce37eb4'),
        (0x2dc990,0x2dc9a8,'2514d6567752cea2fb3255856ed41e2cbe5c0e4e276d9f3310a81d895ae012af'),
        (0x2dbd90,0x2dbdac,'e6e81f90b768228012c77b0ff794a4068ceb0187abc59dc45364159a238dd8ae'),
        (0x2dbdb0,0x2dbde0,'d75aad57b1cf26127f35bdf8a0eec8259d79c6b1cb8cbbabe269467dac4546ef'),
        (0x2d9330,0x2d9360,'d75aad57b1cf26127f35bdf8a0eec8259d79c6b1cb8cbbabe269467dac4546ef'),
        (0x2d9360,0x2da9a8,'41fef5550eda7e9b62d9c20827b3e5e86c3b82b7f4c0061f80ddb90612845297'),
        (0x2cd020,0x2d18d4,'618be48d56debcac8ef3fd9d80ae23483f060a7939fc8d450772d444e91e7d61'),
        (0x2d9310,0x2d932c,'e6e81f90b768228012c77b0ff794a4068ceb0187abc59dc45364159a238dd8ae'),
        (0x2da9b0,0x2dab58,'e9616a7a67d9711340f671a0f247d5bad039f18c4614995d8d9e91f983decbe2'),
        (0x205a80,0x205ab0,'d75aad57b1cf26127f35bdf8a0eec8259d79c6b1cb8cbbabe269467dac4546ef'),
        (0x205ab0,0x20682c,'aa69cc235004b48714a683650ab5a9189d669eac794317ac821edf1093daa324'),
        (0x205a60,0x205a7c,'e6e81f90b768228012c77b0ff794a4068ceb0187abc59dc45364159a238dd8ae'),
        (0x206830,0x206960,'7be64c4e2c5d1ed6a7e2d3424592ee0b05cc0fec84bfef3a11f2d22f1554822c'),
        (0x2c76d0,0x2c81c4,'8e35c67b4a3f83e1793832f99d7cf6d43aa4292882c3e182bf72bffd678aa541'),
        (0x2c81d0,0x2c8320,'1b6a7bd130edea1272195affad652d65c9d8fe9f21a3546fb0c814809c69e364'),
        (0x2143e0,0x2144bc,'215972741f87f32b518d7c668445cb57c11ce3579448a56bd30bafb4105339c5'),
        (0x2144c0,0x21458c,'829751c1cd99cfd0235237d0754e72e969869a5038b4f62a5204649119a219e0'),
        (0x2f2d70,0x2f2de0,'30a2923a7aa2d9c89d24b8fddcff01006d3ea2774afe0c390c7e215679e472f7'),
        (0x2f2df0,0x2f2fd8,'31595a371060732f2e757e834be15d505f0ff4ed0481120cff61ced5bc0635b1'),
        (0x2e95d0,0x2e9720,'3d4199cb56a1ada2fd313ab844f77adf78b1686446a97d85c1780e3285656d7d'),
        (0x2e9ce0,0x2ea348,'d0ec213ab5533e4c4f1dd9c34b414fe126f17660a0e92d059d6dd17c4e21b707'),
        (0x2ead10,0x2eb3a0,'e4ab374a171d1c857267fd7fac3c7f35cf92c1f81d30f0988b3d85bf5ccbd78b'),
        (0x357900,0x357960,'462a6e78b8921044746093d1279f2e1fa139896c45bacee515d4504e946a0a24'),
        (0x30ed90,0x30ee00,'99dd9b859b2f98960359a3d9f43a0f78761f81ebe207c707331e026607154550'),
        (0x30ee00,0x30f0c0,'15d8bbc42064f45964cc032ed2db9a95beeb2530a0c24e0ead872528eae453e4'),
        (0x306950,0x306f84,'1d86e9d29099ea34987865d5e92e9912b090428d65f9c26e8e2b9ba62d18f156'),
        (0x30ce70,0x30ceb0,'953419ce510a7ca81e1955b302db4fe8eb4b944b91967725615a3ebbb8d3c2f6'),
        (0x239f70,0x239fa8,'c3849227f4e9e451d7b72f6b758661de4ed53015868faa6d5fb18a88f95167f9'),
        (0x239fd0,0x23a000,'941b168de034258eff47866950005885121fdd8a9f4de7cd8f57bc04c3b5bb94'),
        (0x25aa10,0x25aa40,'d75aad57b1cf26127f35bdf8a0eec8259d79c6b1cb8cbbabe269467dac4546ef'),
        (0x25aa40,0x25be00,'0df64feffaf54ce1d51dfb4bd13f04e1a4160b445284eb62d1aa8b3618c9ff04'),
        (0x2f0de0,0x2f0e70,'1898503143ca6ccba1d99bc209d3a7fd3c9e9a0babc361b702fb1267df6eb301'),
        (0x29af80,0x29b2c4,'ef3a9f1a6cae966ab8a687451e72ac175d88b8e9883d09449949ca5c3389d0bf'),
        (0x29c990,0x29cb18,'361b601c33bd3f57fd190aef95f4716cd08b5bbdf401f852b6e80eb96933e9e2'),
        (0x2bef20,0x2bf0dc,'2562b94cada2f70f71933a401cc6eacc7c1eb0a71f60dafc6b7179d58fdb0fb7'),
        (0x248fc0,0x24926c,'c80605a259562aba190d4009dda4f3f913e461937e47c6e4dca959f8f81624cc'),
        (0x213fd0,0x2140f4,'929eb425f5d11c4d38fd2c0d2c18b8bad91718a8fe3845ba40df7e08a9392768'),
        (0x214e40,0x2150f0,'c83bfc3a70b84ef6ccb1de817de8490cc12fe95f428f6519c4f3ed0700c9af5f'),
        (0x214c40,0x214e40,'d9106bc728de471b557f7f136424ebe55e3d59a694b9ff7f0ba9e91807180a0e'),
        (0x22e650,0x22f0d4,'4b641be6f4d9b2450646a7b89b06f0b1446d290c4e91ccbd192fbb16a76bd9d9'),
        (0x2151d0,0x2153a0,'14b0ca18d4340479715e85d75072aa68e5ae9307b54cbcb8e86313628685308a'),
        (0x2150f0,0x2151d0,'a963b19e1061c2400324b4440cef7d6da40124ddafaad3e84a10ab96b882944b'),
        (0x1fe8a8,0x1fe95c,'5fc3d4add9925c4b8b5607379880ef718cf0f6e5d45a0c9f180a9ae80ff3b6a8'),
        (0x358490,0x358690,'9a1c658a34ce5474bcf366b3d05296b4eb6de58357684808a64cdcbe12b91b53'),
        (0x216e40,0x216f4c,'422ab7a80ab28b304ccdcd915c85f68fd5077dece21a302982f240d80845ccc8'),
        (0x216f50,0x21730c,'af06d3a8eedadf39b9354c69400b970b8f4dccd52635e29d7bd0d7f36e412b3e'),
        (0x2262b0,0x22b880,'42becdf1b5bdeadbfb822b3c240d8e75ad43bf508ab3f159cd9bd0bf6229c009'),
        (0x22f9c0,0x22fa50,'2cf24b9daf5f4adce5fd1b6ae8fe971ad6f84a96f45050c3779f9b6099f96030'),
        (0x22f170,0x22f8f8,'d28eba3b2b21e88ad0fd553a8008ac0486e2b8d90dd163d3a5d1ebc40532d14d'),
        (0x22f140,0x22f170,'b14ea2dd33db827c45cf9ac361606325a10e5c467c73a3901a3e5bfc33f491ed'),
        (0x247f90,0x248284,'da53050cf4ec8fca89e763dba6a935eeb0cb7964b76c859d636a847ddf743dbb'),
        (0x2476b0,0x247ae8,'f00f556c4ad7f0cd1d210a74870318cbb59d4533b0c484f9fa774fe01f43aee8'),
        (0x247650,0x2476b0,'a12576781a99ee12242a602803b5241bea9a20cb81ce9d1cfb97194a851bb47a'),
        (0x23de88,0x23dec8,'6d9b198f4061a2ec3918dc9b4da1516ff956bc236bb8756946ccad150f929fe5'),
        (0x247ec0,0x247f88,'9e9bb6a71cea9384182083a2f88a391f0c2720f6825b6f91b85612d58acc3fda'),
        (0x248290,0x248324,'bfef5d5d6cdacb1b7602f2307178d03e47bd011efd79c9afb4f8208af13d56a5'),
        (0x2371dc,0x2372d0,'8dd9f9b8eebc36de6371dc3273828d44f6ac31eaaba90739ab75948ca85677fc'),
        (0x1fedc0,0x1fee10,'fdb88de8c0b4b42b3c9418d58b9b2337c1673c2a9e985d7f989d08178ef302ab'),
        (0x20f240,0x20f2bc,'49f956ae5d5022771849640497bdcaf22d18020f73003fce8799bd380fa53720'),
        (0x23c240,0x23c2d0,'d6dc553b8d93e236168880a6b9377c26b44cd7ca20d62db46752698eadb3748c'),
        (0x248560,0x248630,'087896e56eac45934cf3164ba2faff490cb51fc343c37c9fc097326605d2f2f8'),
        (0x216cb0,0x216d60,'daa75aec7b1a3f27c1abbdb274643fce6f62c12fdf7fb89df10faaf79e9e8dcf'),
        (0x1fb6a0,0x1fb7b8,'f96d882aa5932820bbfa87dd754afdb0af1880d0103d6620f464121daebddbef'),
        (0x1fc9e0,0x1fcb3c,'298314148123e9f3b310e7783d7a25276b80e185063f494e2e4f81b6ece2db9c'),
        (0x1fc860,0x1fc9e0,'9f3edcf75b36591158ad4503787d76d624703895d1c1e91f6cae80336ee9e694'),
        (0x1fc6d0,0x1fc860,'5188f268e63a6fc632bb955dbbe4c869049a20939f9efd87cea52ab70d129040'),
        (0x1fde10,0x1fde88,'450ccf730c26a56abdf87e8477109715187f8b4789c33f427db6e2e450b7a711'),
        (0x2120a0,0x212950,'1d0b09b9fcd53a5a9c33f52892d1b3567324a59e30c8c3b694561a1d4bfee4b0'),
        (0x211fb0,0x2120a0,'6b7507c0acedfc7e0b9f3c6c45748d8b362e8e2f060f6c245c92359d41d721e7'),
        (0x1fbfb0,0x1fc030,'5e88b2604b471df076c27747f18e511a0ac356176fdaf7141265e3a73d37f563'),
        (0x1fe1e0,0x1fe258,'eba5026c9638d34728fca0b3a214b42260e0d1ddc2ad15898868421fdcda38cc'),
        (0x1fc244,0x1fc2f8,'7d4999bd72dd0fe0dfb842f4a9b5dbb7c5cf842e25e17c9f402c3efec0507a32'),
        (0x1fc4f0,0x1fc5b0,'ad66a86513e043a904f04f642285a822d4de2a507ec1bb6f583db136e0f909ad'),
        (0x1fc3dc,0x1fc488,'8c39e52c381ef962f8254aebfeea07d0bf18495f250d0cbb27daf467f4332bd4'),
        (0x1fc5b0,0x1fc6d0,'0be1d90990257736edb6dc07a3633c02ce35132d9497196462b64059c568bdc4'),
        (0x1fc030,0x1fc180,'d9942d91ce4b4825dd6f9dd63da6bf26dc26877ca314d649d66fcc504747c56a'),
        (0x1fde88,0x1fe024,'1e92afb7357d54a4a5f8a45d96077acbf7e47ad6ae0ac78e9ebd282990ee497d'),
        (0x1fbdcc,0x1fbdf4,'6612d3b4b406e0366f2c9856ea4a99203f3ed31136be8aae1e2f5d628a4501ae'),
        (0x1fbe60,0x1fbfa8,'ee83e8648a1d0d5faf59d6aec5bd02be226bc8a6146f0a5e722a10025478fe70'),
        (0x1fcb40,0x1fd110,'c14c93ed7c1656ac4eb37881e5e50d0e65b36bd9cb9dba85f91c05d36e569011'),
        (0x216d60,0x216e40,'a83d8cda7834128332dc053209444b9653c52f2de21aa30da27df38498c1243e'),
        (0x1fb1e4,0x1fb204,'6d021a981a3d5b2de8da07172b9af49bf894ea3fb32a31bd238e794aadc18074'),
        (0x1fb2fc,0x1fb31c,'6d021a981a3d5b2de8da07172b9af49bf894ea3fb32a31bd238e794aadc18074'),
        (0x1fb42c,0x1fb44c,'6d021a981a3d5b2de8da07172b9af49bf894ea3fb32a31bd238e794aadc18074'),
        (0x1fb624,0x1fb644,'6d021a981a3d5b2de8da07172b9af49bf894ea3fb32a31bd238e794aadc18074'),
        (0x1fb3f4,0x1fb41c,'fb12a81f4e63c52e83b4c56253bb7a0e258afe3f5f036ee5f9e0ecb19cd1a0c6'),
        (0x1fb5ec,0x1fb614,'fb12a81f4e63c52e83b4c56253bb7a0e258afe3f5f036ee5f9e0ecb19cd1a0c6'),
        (0x1fb87c,0x1fb8a4,'fb12a81f4e63c52e83b4c56253bb7a0e258afe3f5f036ee5f9e0ecb19cd1a0c6'),
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
        # Mode 1: a single REFE installing a second 410-pair VU0 program (no
        # UNPACK/MSCAL); table entry and the whole resource are fixed.
        (0x313100,0x313df0,'dddda5466feacf82fea1794ebdbb071f6e2a631f48501fd15a8417b29615f58c'),
        (0x314138,0x314140,'62295285f66d7820b28099ac97093512c3520f3c7fbf62d9014044d627be9aa3'),
        (0x217550,0x2175c0,'bf29f73fdf9756c87ee0b7b11aaccb586cb29e0fb0653934452e96586c48b694'),
        (0x215870,0x2159c0,'c3cd1b33f97be33016d939dcf099aa81f1dddc53735f661478db065e35ed1c47'),
        (0x215640,0x2156c8,'eaaddd85c3dde1abde3f12e8cc605374ea14f6dae53cd29c9244b8294ed1c676'),
        (0x2076a0,0x2077e0,'3dbd062a459cbdf356c0f99f7dca46d7dcdcd9b593ebfb56fd52e451e201dbd7'),
        (0x2075a0,0x2075e0,'b4f2072552d30100b174620bc445c64fa971d56c9776539660d3e632b2a939b2'),
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
    return ('0030ce84 3c011000 00000000\n'
            '0030ce8c 8c230000 00000000\n'
            '0030ce90 00621021 00000000\n'
            '0030ce94 3042000f 00000000\n'
            '0030ce98 00021100 00000000\n'
            '0030ce9c 02021021 00000000\n'
            '00306a10 3c011000 00000000\n'
            '00306a14 00102402 00000000\n'
            '00306a18 8c230000 00000000\n'
            '00306a1c 00823021 00000000\n'
            '00306a28 000310c0 00000000\n'
            '00306a2c 00431023 00000000\n'
            '00306a30 00468821 00000000\n'
            '00306a38 02338821 00000000\n'
            '00306b84 3c011000 00000000\n'
            '00306b8c 8c340000 00000000\n'
            '00306bb8 3c011000 00000000\n'
            '00306bbc 8c220000 00000000\n'
            '00306bc0 00541023 00000000\n'
            '00306bc4 2c41002b 00000000\n'
            '00306bcc 3c011000 00000000\n'
            '00306bd4 8c340000 00000000\n'
            '00249830 ac22e000 00000000\n'
            # Preserve the empty child writer's original CPU flush, then run
            # its real scalar completion. Padding holds the relocated epilogue.
            '002075c8 dfbf0000 0c081d60\n'
            '002075cc 03e00008 00000000\n'
            '002075d0 27bd0010 dfbf0000\n'
            '002075d4 00000000 03e00008\n'
            '002075d8 00000000 27bd0010\n'
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
            # Native model draws own copied geometry, transforms and textures
            # before the real scalar scene completion; no mesh DMA enqueue.
            '00239fec 0c0807c8 0c087268\n'
            # Publish in the outer producer before a transport job can exist;
            # enter the original scalar completion with the owning object.
            '002dbdcc 0c0807c8 0c0b6f64\n'
            '002d934c 0c0807c8 0c0b64c4\n'
            '00205a9c 0c0807c8 0c081698\n'
            '0022f15c 0c0807c8 0c08bc48\n'
            '00247698 0c0807c8 0c091d8c\n'
            # Explicit particle records are already published synchronously.
            # Remove both device polls, without fabricating completion bits.
            '0022ee50 1a600009 10000009\n'
            '0022f048 1a600009 10000009\n'
            # Animated packet UVs are rewritten natively from their records at
            # 219210; the scratchpad streaming section (transfers/polls) is skipped.
            '00219214 184000d2 100000d2\n'
            '0025aa2c 0c0807c8 0c087268\n'
            '002f3710 0c0bbce4 0c087268\n'
            '002ec9e0 0c0bbce4 0c087268\n'
            # A native decoder owns the prepared media buffer. Preserve its
            # original CPU reset/tag construction; never create an IPU DMA job.
            '002bc218 afa2003c 080af0b4\n'
            '002bc120 3c011001 080af074\n'
            '002bc124 ac20b420 00000000\n'
            '001ef864 0c07bd8a 00000000\n'
            '001ef86c 0c07c10e 00000000\n'
            '001efa40 0c0504a8 0807beb8\n'
            '001efae8 ac430000 03e00008\n'
            '001efaec 0000282d 27bd0010\n'
            '001f048c 8ca30000 00000000\n'
            '001f049c 00641824 00000000\n'
            '001f04a4 00661825 00000000\n'
            '001f04a8 aca30000 00000000\n'
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
    if name=='mod003':
        overlay_adaptations(name,data)
        return 'native 0x40b040\nnative 0x3d93f4\nnative 0x3d38d4\nnative 0x3d363c\nnative 0x3d364c\nnative 0x3d49b0\nnative 0x447af0\nnative 0x447b84\nnative 0x447d60\nnative 0x447a48\nnative 0x447f50\n'+''.join(f'native 0x{pc:x}\n' for pc in (0x3e5ca4,0x3e5ca8,0x3e5cac,0x3e5cb4,0x3e5cb8,0x3e5cc0,*range(0x3e7edc,0x3e7f38,4),*range(0x3e7fa0,0x3e7ffc,4)))
    if name=='mod002':
        _,segments,_=elf_parts(data);lo,hi=0x3622e8,0x362338
        s=next(s for s in segments if s[0]==1 and s[2]<=lo<hi<=s[2]+s[4]);at=s[1]+lo-s[2]
        if hashlib.sha256(data[at:at+hi-lo]).hexdigest()!='6d87622143495b5e67a67c1a1655a2eef28d48c7c1d78309ea18a3ad3e6f45ed':raise ValueError('USA debug-font publication caller mismatch')
        overlay_adaptations(name,data)
        return 'native 0x362328\nnative 0x3721a4\nnative 0x3721b0\n'
    if name!='mod001':return ''
    overlay_adaptations(name,data)
    return ''.join(f'native 0x{pc:x}\n' for pc in (
        *LEAFLESS_QUERY_SITES,
        *FULL_SINE_SITES,
        *FULL_COSINE_SITES,
        *EULER_QUATERNION_SITES,
        *POSITIVE_EULER_SITES,
        *DOF_MATRIX_SITES,
        *INVERSE_PIVOT_SITES,
        *OBJECT_UPDATE_POINT_SITES,
        *OBJECT_QUERY_POINT_SITES,
        *OBJECT_LINEAR_SITES,
        *OBJECT_CHANGE_SITES,
        *HERMITE_VECTOR_SITES,
        *SQUAD_SITES,
        *PARTICLE_PUBLICATION_SITES,
        0x219210,
        0x22f160,
        0x247ec0,
        0x247700,0x247a90,0x247ac0,0x24769c,
        0x2dbde0,0x2dbeb4,0x2dc6c8,0x2dbdd0,0x2dc700,
        0x2d9360,0x2cd020,0x2d18a0,0x2da980,0x2d9350,0x2da9b0,
        0x205ab0,0x206808,0x205aa0,0x206830,
        *range(0x252ec8,0x252f24,4),0x252fdc,0x252fe0,0x252fe4,0x252fec,0x252ff0,0x252ff8,
        0x2bf040,0x2bf058,
        0x2371e8,0x2371ec,0x2371f0,0x2371f4,0x2371fc,0x237200,0x237204,0x237208,
        0x23720c,0x237210,0x237214,0x237218,0x237224,0x237260,0x237264,0x237268,0x23726c,
        0x237270,0x237274,0x237278,0x23727c,0x237280,0x237284,0x237288,0x23728c,0x237290,
        0x23729c,0x2372b8,0x2372bc,0x2372c0,0x2372c4,0x2372c8,
        0x2075bc,0x2075cc,0x30ce84,0x306a10,0x306b84,0x306bc4,0x306c58,
        0x30ee00,0x30ef20,0x30ef2c,0x30ef44,0x30ef78,0x30efb8,0x30eff4,0x30f028,0x30f070,0x30ed90,
        0x214540,0x2e9640,0x2e970c,0x2e95d0,0x2e9e90,0x2ea154,0x2ea338,0x2ead10,0x2eae90,0x2eb07c,0x2eb284,0x2eb2ec,0x2eb300,0x2eb320,0x2eb340,0x2eb378,
        0x29e1a0,0x2c3cb4,0x29cc30,0x29cd7c,0x29cf14,0x29cf30,0x29cc00,0x29e160,
        0x208a94,0x29af80,0x29b2a8,0x29caf8,0x2205c0,0x220ce0,0x29e43c,0x29e444,
        0x22106c,0x221294,0x221308,0x221540,0x221704,0x221718,0x221748,0x2088f0,
        0x221698,0x22169c,0x21c9dc,0x21c9e0,0x21c9e8,0x21c9ec,0x21ca1c,0x21ca20,0x21ca24,0x21ca28,0x2cb490,0x2f3714,0x2ea9a0,
        0x2e74f0,0x2e7520,0x2e7550,0x2e757c,0x2e75a0,0x2f0de0,0x2e83e4,0x2ec9e4,0x2ec308,0x2ec320,
        0x2efe40,0x2effe8,0x2eb3a0,0x2eb5c4,0x2ebac4,0x2ebb0c,0x2ebb58,0x2ebb9c,0x2ebba8,0x2bbfc0,0x2bc1d0,0x2bc1e0,0x2ba670,0x1ef6f8,0x1ef914,0x1f0438,0x1f04a0,0x1efa30,
        0x1f4590,0x1f4940,0x2bdac0,0x2bd9b0,0x2bd5d8,0x2be3d0,0x2bea60,0x239f70,0x239ff0,0x25aa30,0x25aa40,0x25bad0,0x25bdcc,0x25adbc,0x214388,0x2f2df0,0x2f2fb8,0x2f2d70))+'stub 0x2b90a0\nstub 0x2b9560\nstub 0x1f04c8\n'
