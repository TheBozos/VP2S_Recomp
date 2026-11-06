"""USA-only guards for retiring the SDK's firmware syscall-table discovery.

No instructions or load bytes are changed. These are port-owned native binding
declarations, deliberately separate from general SetSyscall support.
"""
import hashlib
from vp2_bozo_prepare import elf_parts

INSTALLER = (0x141448, 0x141544)
QUERY = (0x141410, 0x141444)
GUARDS = {
    (0x141378, 0x141444): 'e0986ae7dae0f8e7c70713eb78d9c5f8fc026874de14dbcfbfca790c983be506',
    INSTALLER: 'e3c497dbca0fa7aa92e3ec912159cbac2c0e5c24819b259ac207a22e8c0fbdf4',
    (0x141558, 0x1415ac): 'f99915ebc05eeca2deb3b798fde342e27ee457084bc30313a685c6ae54834328',
    (0x1492a8, 0x1492c0): 'c0a2bc71d842355d9d060d18967d916f5d3e776a0cf84296253406ca5f7a489a',
}


def bindings(data):
    _, segments, _ = elf_parts(data)
    for (lo, hi), expected in GUARDS.items():
        matches = [s for s in segments if s[0] == 1 and s[2] <= lo < hi <= s[2]+s[4]]
        if len(matches) != 1:
            raise ValueError(f'native syscall-bootstrap guard not image-backed: {lo:x}')
        segment = matches[0]
        start = segment[1]+lo-segment[2]
        if hashlib.sha256(data[start:start+hi-lo]).hexdigest() != expected:
            raise ValueError(f'USA syscall-bootstrap signature mismatch: {lo:x}')
    sites = sorted(set(range(*INSTALLER, 4)) | set(range(*QUERY, 4)))
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
        if not control:
            declarations.append(f'native 0x{pc:x}\n')
    return ''.join(declarations)
