"""Convert USA startup key constants and lower its EE hardware bridge offline.

Only the user's ELF supplies constants. No key or game-derived output is checked in.
The port's generated EE code calls a native function on the original Csi seed.
"""
import argparse
import importlib.util
import struct
import sys
from pathlib import Path


def write_changed(path, data):
    if not path.exists() or path.read_bytes() != data:
        path.write_bytes(data)


def lower(text, name):
    marker = '// VP2S native bootstrap lowering v2'
    if marker in text:
        return text
    include = '#include "ps2_recomp_calls.h"'
    assert text.count(include) == 1, name
    text = text.replace(include, include + '\n' + marker + '\n'
                        'bool vp2sBootstrapEnabled();\n'
                        'void vp2sBootstrapKey(uint8_t*, R5900Context*);')
    if 'label_10087c:' in text:
        # The carved 0x100800 entry ends before the seed RPC. Its existing
        # fallthrough dispatch is 0x1008b8; the full main can jump there locally.
        jump = ('goto label_1008d4;' if '// 0x1008d4:' in text
                else 'ctx->pc = 0x1008b8u; return;')
        text = text.replace('label_10087c:', 'label_10087c:\n'
                            '    if (vp2sBootstrapEnabled()) { ' + jump + ' }')
    if '// 0x1008d4:' in text:
        assert text.count('label_1008b8:') == 1, name
        text = text.replace('label_1008b8:', 'label_1008b8:\n'
                            '    if (vp2sBootstrapEnabled()) goto label_1008d4;')
        if 'label_1008d4:' not in text:
            text = text.replace('    // 0x1008d4:', 'label_1008d4:\n    // 0x1008d4:')
        def boundary(pc):
            # The full main labels every instruction; carved entries label only
            # branch/resume targets. Keep the next instruction's label outside
            # the comparison-only scope in either representation.
            pos = text.index('    // 0x' + pc + ':')
            label = 'label_' + pc + ':\n'
            return pos-len(label) if text[:pos].endswith(label) else pos
        begin, end = text.index('    // 0x1008e4:'), boundary('1008e8')
        ack = text[begin:end]
        text = text[:begin] + '    if (!vp2sBootstrapEnabled()) {\n' + ack + '    }\n' + text[end:]
        begin, end = text.index('    // 0x100908:'), boundary('100930')
        original = text[begin:end]
        text = text[:begin] + ('    if (vp2sBootstrapEnabled()) {\n'
                               '        vp2sBootstrapKey(rdram, ctx);\n'
                               '    } else {\n') + original + '    }\n' + text[end:]
    return text


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--elf', type=Path, required=True)
    ap.add_argument('--recomp', type=Path, required=True)
    ap.add_argument('--out', type=Path, required=True)
    args = ap.parse_args()
    elf = args.elf.read_bytes()
    code = elf[0x48b98:0x48bf0]
    pairs = list(struct.iter_unpack('<II', code))
    expected = [0x800002ff, 0x8100089e, 0x810010e2, 0x8080089e,
                0x808010e2, 0x8040089e, 0x804010e2, 0x8020089e,
                0x802010e2, 0x400002ff, 0x000002ff]
    if [upper for _, upper in pairs] != expected:
        raise ValueError('Unexpected USA startup algorithm; refusing native lowering')
    constants = [lower for lower, _ in pairs[:8]]
    args.out.mkdir(parents=True, exist_ok=True)
    write_changed(args.out / 'tables.bin', b'VPBK' + struct.pack('<I8I', 32,
                  *constants[0::2], *constants[1::2]))
    # Original lifted game code is available to the offline numeric checker only.
    sys.dont_write_bytecode = True
    lifter = Path(__file__).resolve().parent.parent / 'external/PS2Recomp/tools/vu1_lift.py'
    spec = importlib.util.spec_from_file_location('bootstrap_lifter', lifter)
    lift = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(lift)
    image = code + bytes(16384 - len(code))
    reference, _ = lift.generate(image, dict(hash=lift.code_hash(code), pairs=11,
                                 function='bootstrap_reference', entries=[0], targets=[], unit='VU0'))
    write_changed(args.out / 'reference.cpp', reference.encode())
    names = ['FUN_00100230_0x100230.cpp', 'entry_00100800_0x100800.cpp',
             'entry_001008b8_0x1008b8.cpp']
    for name in names:
        path = args.recomp / 'main' / name
        text = path.read_text()
        write_changed(path, lower(text, name).encode())
    print('Native startup: converted affine constants and lowered three EE entries')


if __name__ == '__main__':
    main()
