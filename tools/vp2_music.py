"""Compile VP2 USA's music algorithms and tables offline from the user's main ELF.

The output is game code with typed coefficient/spectrum/history buffers, not a VU
execution environment. No packets, code images or PS2 address space are consumed
by the generated native decoder. Game-derived outputs stay in the build tree.
"""
import argparse
import hashlib
import importlib.util
import json
import re
import struct
import sys
from pathlib import Path
from vu1_native import return_targets

ap = argparse.ArgumentParser(description=__doc__)
ap.add_argument('--elf', type=Path, default=Path('build_usa/elf/main.elf'))
ap.add_argument('--out', type=Path, default=Path('build_usa/native_music'))
args = ap.parse_args()
root = args.out
root.mkdir(parents=True, exist_ok=True)
elf = args.elf.read_bytes()
ph = struct.unpack_from('<I', elf, 28)[0]
segments = [struct.unpack_from('<8I', elf, ph + 32*k) for k in range(struct.unpack_from('<H',elf,44)[0])]
def at(address, size):
    for kind, offset, va, _, filesz, *_ in segments:
        if kind == 1 and va <= address and address + size <= va + filesz:
            return elf[offset+address-va:offset+address-va+size]
    raise ValueError(f'ELF does not contain {address:x}+{size:x} (USA main ELF required)')

# Only this offline tool reads the fixed music upload block. It does not follow a
# running game's chains. Fail if another disc/library build has a different shape.
code = bytearray(16384)
upload = at(0x14a310, 0x2420)
p = 0
while p + 4 <= len(upload):
    w, = struct.unpack_from('<I', upload, p)
    p += 4
    cmd, num, imm = w >> 24 & 127, w >> 16 & 255, w & 65535
    if cmd == 0:
        continue
    if cmd != 0x4a:
        raise ValueError(f'unknown music upload command {w:08x}')
    size, dest = (num or 256)*8, imm*8
    if p+size > len(upload) or dest+size > len(code):
        raise ValueError('truncated music code image')
    code[dest:dest+size] = upload[p:p+size]
    p += size
module_path = Path(__file__).resolve().parent.parent / 'external/PS2Recomp/tools/vu1_lift.py'
spec = importlib.util.spec_from_file_location('music_lifter', module_path)
lift = importlib.util.module_from_spec(spec)
sys.dont_write_bytecode = True
spec.loader.exec_module(lift)
targets = return_targets(code)
programs = [('dequantize', 0x6e0), ('window', 0x1410), ('synthesize', 0x1690)]
out = ['#include "vp2s_music_math.h"\nnamespace vp2s::music {\n']
references = {}
for name, entry in programs:
    text, _ = lift.generate(code, dict(hash=lift.code_hash(code), pairs=2048, function=f'music_{entry:x}',
                                      entries=[entry], targets=targets, unit='VU1'))
    references[entry] = text
    if name == 'window':
        out.append('''void window(const Block &spectrum, const Tables &tables, Block &result) {
    for(int row=0; row<32; ++row) for(int column=0; column<8; ++column) {
        Vector sums[4];
        for(int group=0; group<4; ++group) {
            Vector acc = _mm_mul_ps(spectrum.at(column*32+group*8), tables.windowMatrix.at(row*8));
            for(int k=1; k<8; ++k)
                acc = codecSum(acc, _mm_mul_ps(spectrum.at(column*32+group*8+k), tables.windowMatrix.at(row*8+k)));
            sums[group] = acc;
        }
        auto lane=[](Vector v,int k){return laneValue(v,k);};
        auto add=[](float a,float b){return codecSum(a,b);};
        Vector v = _mm_setr_ps(add(add(lane(sums[0],1),lane(sums[0],0)),add(lane(sums[0],3),lane(sums[0],2))),
                              add(add(lane(sums[1],2),lane(sums[1],1)),add(lane(sums[1],0),lane(sums[1],3))),
                              add(add(lane(sums[2],3),lane(sums[2],2)),add(lane(sums[2],1),lane(sums[2],0))),
                              add(add(lane(sums[3],0),lane(sums[3],3)),add(lane(sums[3],2),lane(sums[3],1))));
        result.at(row*8+column) = _mm_mul_ps(v, _mm_set1_ps(0.25f));
    }
}''')
        continue
    body = text[text.index('        n0:'):text.index('    done:')]
    if name == 'synthesize':
        # Replace the scheduled, prefetched integer-packing loop with its direct
        # bounded operation. Dead conversion continuations are removed below.
        a = body.index('            iReg = normalizeOperand(std::bit_cast<float>(0x46fffe00u));')
        b = body.index('goto done;', a) + len('goto done;')
        body = body[:a] + '''
            for (Vector &value : result)
                value = _mm_castsi128_ps(_mm_cvttps_epi32(_mm_max_ps(_mm_set1_ps(-32768.f),
                                             _mm_min_ps(_mm_set1_ps(32767.f), value))));
            goto done;
''' + body[b:]
        chunks = re.split(r'(?m)^\s*(n\d+):', body)
        blocks = {chunks[i]: chunks[i+1] for i in range(1,len(chunks),2)}
        order = list(blocks)
        edges = {}
        for k,label in enumerate(order):
            lines = [s.strip() for s in blocks[label].splitlines() if s.strip()]
            edges[label] = set(re.findall(r'goto (n\d+);',blocks[label]))
            terminal = lines[-1].startswith('goto ') and not lines[-2].startswith('if ')
            if not terminal and k+1 < len(order):
                edges[label].add(order[k+1])
        reached, todo = set(), ['n0']
        while todo:
            label = todo.pop()
            if label in reached: continue
            reached.add(label)
            if label not in blocks: raise ValueError('unmapped synthesis continuation '+label)
            todo.extend(edges[label] - reached)
        body = '\n'.join(label+':'+blocks[label] for label in order if label in reached)
    signature = {'dequantize': 'const Quantized &input, const Tables &tables, Block &spectrum',
                 'window': 'const Block &spectrum, const Tables &tables, Block &result',
                 'synthesize': 'const Tables &tables, Block &history, Block &result, float gain'}[name]
    lines = [f'void {name}({signature}) {{', 'const Vector kVf0 = _mm_setr_ps(0,0,0,1);',
             'Vector acc = _mm_setzero_ps();', 'float iReg=0, q=0, qPending=0;', 'uint32_t jrTarget=0;']
    lines += [f'Vector vf{k} = '+ ('_mm_setr_ps(gain,gain,gain,gain+1)' if name == 'synthesize' and k == 3 else '_mm_setzero_ps()')+';' for k in range(1,32)]
    lines += [f'int32_t vi{k} = 0;' for k in range(1,16)]
    classes = sorted(set(re.findall(r'\bmac_[0-9a-f]+', body)))
    lines += ['uint32_t ' + ','.join(c+'=0' for c in classes) + ';'] if classes else []
    def load(expression):
        e = expression.strip()
        if name == 'dequantize':
            if e == '0 + 256': return '_mm_castsi128_ps(_mm_loadu_si128(reinterpret_cast<const __m128i*>(input.header.data())))'
            if e == 'vi2': return '_mm_castsi128_ps(_mm_loadu_si128(reinterpret_cast<const __m128i*>(input.coefficients.at(vi2-264).data())))'
            if re.fullmatch(r'0 \+ (?:49[6-9]|50[0-9]|51[01])',e): return f'tables.rotations.at({int(e.split()[-1])-496})'
            if e == 'vi3 + 512': return 'quantizerRow(tables, vi3, "global scale")'
            if re.fullmatch(r'vi[345678] \+ (?:512|640)',e):
                vi, _, offset = e.split()
                neighbor = ', true' if e == 'vi8 + 512' else ''
                return f'quantizerRow(tables, {vi}+{int(offset)-512}, "{e}"{neighbor})'
            if e in ('vi4','vi5'): return f'spectrum.at({e})'
        elif name == 'window':
            if e == 'vi2': return 'trailingPrefetch(spectrum, vi2)'
            if e == 'vi3': return 'windowRow(tables, vi3-768)'
        else:
            if e == '0 + 0': return '_mm_set1_ps(gain)'
            if e in ('vi1','vi2'): return f'result.at({e}-768)'
            if e.startswith('vi3 + '): return f'tables.synthesis.at({e}-256)'
            if e.startswith('vi6 + '): return f'tables.synthesis.at({e}-256)'
            if e == 'vi7 + 512': return 'history.at(vi7)'
        raise ValueError((name,'load',e))
    def store(match):
        mask, e, value = match.groups()
        if name == 'dequantize' and e in ('vi1','vi6','vi7','vi12'): target=f'spectrum.at({e})'
        elif name == 'window' and e == 'vi1': target='result.at(vi1-768)'
        elif name == 'synthesize' and e == '0 + 0': return '// fixed sign-selection constant'
        elif name == 'synthesize' and e in ('vi1','vi2'): target=f'result.at({e}-768)'
        elif name == 'synthesize' and e.startswith('vi5 + '): target=f'history.at({e}-512)'
        else: raise ValueError((name,'store',e))
        return f'{target} = lanes<{mask}>({target}, {value});'
    for line in body.splitlines():
        s = line.strip()
        if re.match(r'(?:t \+=|endCycle =|endPc =|lastMac =|lastCur =|hasFmac =|landDi\()',s): continue
        if s.startswith('default: return kUnknownTarget;'): s = 'default: throw std::runtime_error("invalid music quantizer branch");'
        s = re.sub(r'normalizeOperand\((std::bit_cast<float>\(0x[0-9a-f]+u\))\)',r'\1',s)
        s = s.replace('i16(', 'int16_t(').replace('put<','lanes<')
        s = re.sub(r'divide\((.*), diPending\)',r'ratio(\1)',s)
        s = re.sub(r'm\.ilw\(0 \+ (\d+), (\d+)\)',lambda m: f'input.bands.at({(int(m[1])-257)*4+int(m[2])})',s)
        s = re.sub(r'm\.lq\(([^)]+)\)',lambda m:load(m[1]),s)
        s = re.sub(r'm\.sq<(\d+)>\(([^,]+), ([^)]+)\);',store,s)
        m = re.search(r'fmac<Op::(\w+), R::(\w+), (\d+), kSticky, (true|false)>\(f, ([^,]+), ([^,]+), ([^,]+), acc, ([^,]+), ([^)]+)\)',s)
        if m:
            op,r,mask,flags,old,a,b,classification,_ = m.groups()
            if r.startswith('Bc'): b=f'broadcast<{r[-1]}>({b})'
            elif r == 'Scalar': b=f'_mm_set1_ps({b})'
            elif r != 'Vec': raise ValueError(r)
            fn = dict(Add='sum', Sub='difference', Mul='product', Madd='accumulate', Msub='subtractProduct')[op]
            args = f'{old}, {a}, {b}' + (', acc' if op in ('Madd','Msub') else '')
            if flags == 'true': args += ', &' + classification
            s = s[:m.start()] + f'{fn}<{mask}>({args})' + s[m.end():]
        s = re.sub(r'miniv<R::Vec, 15>\(([^,]+), ([^,]+), ([^)]+)\)',r'_mm_min_ps(\2, \3)',s)
        s = re.sub(r'maxv<R::Scalar, 15>\(([^,]+), ([^,]+), ([^)]+)\)',r'_mm_max_ps(\2, _mm_set1_ps(\3))',s)
        lines.append('    '+s)
    lines += ['done: return;', '}']
    out += lines
out.append('}\n')
native_source = '\n'.join(out)+'\n'
if re.search(r'\b(?:VU1State|VU1Interpreter|Memory|Fmac|Path1|endCycle|kickDone|kSticky)\b|\bm\.|\bstate\.', native_source):
    raise ValueError('unconverted hardware dependency in native music output')
def write_changed(path, data):
    if not path.exists() or path.read_bytes() != data:
        path.write_bytes(data)
write_changed(root/'codec.cpp', native_source.encode())
(root/'reference').mkdir(exist_ok=True)
for entry, text in references.items():
    write_changed(root/'reference'/f'music_{entry:x}.cpp', text.encode())

tables = bytearray(at(0x14ca98, 16*16))
for source in (0x14cba0, 0x14cfa8, 0x14d3b0, 0x14d7b8):
    for a,b in struct.iter_unpack('<2I', at(source, 128*8)):
        tables += struct.pack('<4I', a,b,0,0)  # only the pair is part of this lookup table
for source in (0x14dbc8, 0x14dfd0, 0x14e3d8, 0x14e7e0):
    tables += at(source, 64*16)
for source, size in ((0x14ebf8, 64*16),(0x14f000, 64*16),(0x14f408, 8*16)):
    tables += at(source, size)
assert len(tables) == 14720
write_changed(root/'tables.bin', b'VPMU' + struct.pack('<I',len(tables)) + tables)
write_changed(root/'source.json', (json.dumps(dict(elf_sha256=hashlib.sha256(elf).hexdigest(),
                   code_sha256=hashlib.sha256(code).hexdigest(), table_bytes=len(tables)),indent=2)+'\n').encode())
print('Native music: compiled three game algorithms and converted tables ->', root)
