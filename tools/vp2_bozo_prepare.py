"""Prepare VP2S USA ELF metadata for BozoEngine without changing load bytes.

Inputs are the player's existing extracted ELFs and offline function/SDK maps.
Generated ELFs, identities and reports must stay in an ignored build directory.
"""
import argparse
import csv
import hashlib
import json
import re
import struct
from pathlib import Path

EH = struct.Struct('<16sHHIIIIIHHHHHH')
PH = struct.Struct('<8I')
SH = struct.Struct('<10I')
SY = struct.Struct('<IIIBBH')


def elf_parts(data):
    if len(data) < EH.size:
        raise ValueError('truncated ELF header')
    h = list(EH.unpack_from(data))
    if h[0][:7] != b'\x7fELF\x01\x01\x01' or h[2] != 8:
        raise ValueError('expected little-endian ELF32 MIPS image')
    if h[9] != PH.size or h[11] != SH.size:
        raise ValueError('unexpected ELF table entry size')
    segments = [PH.unpack_from(data, h[5] + i * PH.size) for i in range(h[10])]
    sections = [list(SH.unpack_from(data, h[6] + i * SH.size)) for i in range(h[12])]
    for s in segments:
        if s[0] == 1 and (s[1] + s[4] > len(data) or s[4] > s[5]):
            raise ValueError('invalid load segment')
    return h, segments, sections


def load_identity(data):
    _, segments, _ = elf_parts(data)
    return [(s[2], s[5], s[6], data[s[1]:s[1] + s[4]]) for s in segments if s[0] == 1]


def prepare(data, rows, sdk):
    h, segments, sections = elf_parts(data)
    code = [(s[3], s[3] + s[5], i) for i, s in enumerate(sections) if s[2] & 6 == 6 and s[5]]
    if not code:
        raise ValueError('input needs explicit executable sections')
    entries = {}
    skipped = 0
    for row in rows:
        start, end = int(row['Start'], 0), int(row['End'], 0)
        region = next((r for r in code if r[0] <= start < r[1]), None)
        if region is None:
            skipped += 1
            continue
        if end > region[1]:
            raise ValueError(f'function crosses executable section: {start:x}..{end:x}')
        if start & 3 or end & 3 or end <= start:
            raise ValueError(f'invalid function span {start:x}..{end:x}')
        old = entries.get(start)
        if old is None or end > old[0]:
            entries[start] = (end, row['Name'], region[2])
    # Ghidra exports carved entries inside larger functions. Merge overlapping
    # extents into the primary function, keeping each interior address as a label.
    # BozoEngine otherwise truncates a function at every overlapping STT_FUNC.
    primaries = []
    for start, (end, name, section) in sorted(entries.items()):
        if primaries and section == primaries[-1][3] and start < primaries[-1][1]:
            primaries[-1][1] = max(end, primaries[-1][1])
        else:
            primaries.append([start, end, name, section])
    primary_by_start = {p[0]: p for p in primaries}
    symbols = []
    for start, (end, name, section) in sorted(entries.items()):
        primary = primary_by_start.get(start)
        symbols.append((sdk.get(start, name), start, primary[1] - start if primary else 0,
                        0x12 if primary else 0x10, section))
    # A zero-size SDK function symbol binds native libraries without splitting
    # a containing game function. Never mark symbols outside this image as code.
    for start, name in sorted(sdk.items()):
        region = next((r for r in code if r[0] <= start < r[1]), None)
        if region is None:
            raise ValueError(f'SDK symbol outside executable image: {name}@{start:x}')
        if start not in entries or start not in primary_by_start:
            symbols.append((name, start, 0, 0x12, region[2]))
    # The retail executable sections also contain embedded data/VU images.
    # Describe only the offline EE function-map coverage as executable. Unknown
    # addresses stay unsupported rather than being guessed as runnable code.
    ee_code = []
    for start, end, _, section in primaries:
        if ee_code and start == ee_code[-1][1] and section == ee_code[-1][2]:
            ee_code[-1][1] = end
        else:
            ee_code.append([start, end, section])
    # Replace only metadata; original segment offsets and bytes remain intact.
    if h[13] >= len(sections):
        raise ValueError('invalid section-name table index')
    old_names = sections[h[13]]
    names = (bytearray(data[old_names[4]:old_names[4] + old_names[5]])
             if h[13] and old_names[1] == 3 else bytearray(b'\0'))
    out = bytearray(data)
    new_code = []
    for index, (start, end, section) in enumerate(ee_code):
        original = sections[section]
        n = len(names); names.extend(f'.vp2s.ee{index}'.encode() + b'\0')
        new_code.append((start, end, len(sections)))
        sections.append([n, 1, 6, start, original[4] + start - original[3], end - start, 0, 0, 4, 0])
    for _, _, section in code:
        sections[section][2] &= ~4
    strings = bytearray(b'\0')
    table = bytearray(SY.size)
    for name, start, size, kind, section in symbols:
        section = next((i for lo, hi, i in new_code if lo <= start < hi), section)
        offset = len(strings)
        strings += name.encode('utf-8') + b'\0'
        table += SY.pack(offset, start, size, kind, 0, section)
    # Remove stale symbol tables without renumbering the original sections.
    for s in sections:
        if s[1] == 2:
            s[1] = 0; s[5] = 0
    def add_section(name, kind, payload, link=0, info=0, entry=0):
        out.extend(bytes((-len(out)) % 4))
        offset = len(out); out.extend(payload)
        n = len(names); names.extend(name.encode() + b'\0')
        sections.append([n, kind, 0, 0, offset, len(payload), link, info, 4, entry])
        return len(sections) - 1
    strings_index = add_section('.vp2s.strtab', 3, strings)
    add_section('.vp2s.symtab', 2, table, strings_index, 1, SY.size)
    out.extend(bytes((-len(out)) % 4))
    n = len(names); names.extend(b'.vp2s.shstrtab\0')
    h[13] = len(sections)
    sections.append([n, 3, 0, 0, len(out), len(names), 0, 0, 1, 0])
    out.extend(names); out.extend(bytes((-len(out)) % 4))
    h[6] = len(out); h[12] = len(sections)
    out.extend(b''.join(SH.pack(*s) for s in sections))
    EH.pack_into(out, 0, *h)
    result = bytes(out)
    if load_identity(result) != load_identity(data):
        raise ValueError('prepared ELF changed a load segment')
    return result, dict(functions=len(primaries), entries=len(entries), sdk=len(sdk),
                        entry_addresses=sorted(entries),
                        skipped_outside_code=skipped, code=[[a, b] for a, b, _ in ee_code],
                        original_code=[[a, b] for a, b, _ in code])


def sdk_names(path):
    # Only the actual stub list, not the analyzer's untracked candidates.
    text = path.read_text()
    match = re.search(r'^stubs\s*=\s*\[(.*?)\]', text, re.M | re.S)
    result = {}
    for name, address in re.findall(r'"([^"@]+)@(0x[0-9a-fA-F]+)"', match[1] if match else ''):
        address = int(address, 16)
        if address in result and result[address] != name:
            raise ValueError(f'ambiguous SDK name at {address:x}')
        result[address] = name
    return result


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--build', type=Path, default=Path('build_usa'))
    ap.add_argument('--out', type=Path, default=Path('build_usa/bozo/images'))
    ap.add_argument('--inputs', default='main,mod001')
    args = ap.parse_args()
    if not args.out.resolve().is_relative_to(args.build.resolve()) or args.out.resolve() == (args.build/'elf').resolve():
        raise ValueError('generated images must remain in a separate directory inside the build tree')
    main_image = (args.build/'elf/main.elf').read_bytes()
    if main_image != (args.build/'extract/iso/SLUS_214.52').read_bytes():
        raise ValueError('expected unchanged SLUS-21452 USA main ELF')
    names = args.inputs.split(',') if args.inputs != 'all' else ['main'] + sorted(p.stem for p in (args.build/'elf').glob('mod*.elf'))
    if len(names) != len(set(names)) or any(not re.fullmatch(r'main|mod\d{3}', n) for n in names):
        raise ValueError('invalid/duplicate image names')
    args.out.mkdir(parents=True, exist_ok=True)
    report = {}
    for name in names:
        source = args.build/'elf'/f'{name}.elf'
        data = source.read_bytes()
        with (args.build/'ghidra/export'/f'{name}_functions.csv').open(newline='') as stream:
            rows = list(csv.DictReader(stream))
        prepared, metadata = prepare(data, rows, sdk_names(args.build/'sdk'/f'{name}.toml'))
        target = args.out/f'{name}.elf'
        if not target.exists() or target.read_bytes() != prepared:
            target.write_bytes(prepared)
        metadata['sha256'] = hashlib.sha256(data).hexdigest()
        (args.out/f'{name}.entries').write_text(''.join(f'0x{a:x}\n' for a in metadata['entry_addresses']))
        if name != 'main':
            loads = load_identity(data)
            if len(loads) != 1 or len(loads[0][3]) < 128:
                raise ValueError('overlay needs a complete module header')
            metadata['load'] = loads[0][0]
            metadata['end'] = loads[0][0] + loads[0][1]
            identity = loads[0][3][:64]
            (args.out/f'{name}.identity').write_bytes(identity)
        report[name] = metadata
        print(f'{name}: {metadata["functions"]} primary functions, {metadata["entries"]} entries, {metadata["sdk"]} SDK names; load bytes unchanged')
    (args.out/'manifest.json').write_text(json.dumps(report, indent=2) + '\n')


if __name__ == '__main__':
    main()
