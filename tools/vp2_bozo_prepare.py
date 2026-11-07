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


def overlay_entries(name,data):
    if not re.fullmatch(r'mod\d{3}',name):return []
    header,segments,sections=elf_parts(data)
    def span(lo,hi):
        s=next(s for s in segments if s[0]==1 and s[2]<=lo<hi<=s[2]+s[4]);at=s[1]+lo-s[2]
        return data[at:at+hi-lo]
    if name=='mod001':
        for lo,hi,expected in ((0x358200,0x358a10,'12379d6ed2c2638818509740c6a79294a47daab89c654152a5d9ca80ae864c79'),
                               (0x358a10,0x358a7c,'32428520c5199b05899bf79bd2e0966864f90648beeabb5bfa9548b5417e776e')):
            if hashlib.sha256(span(lo,hi)).hexdigest()!=expected:
                raise ValueError('USA mod001 static initializer signature mismatch')
        if hashlib.sha256(span(0x29e240,0x29e280)).hexdigest()!='1803abf64643018a89bfbd0e71271c70e86d09cdec797d8692b1fdd4602d48cf':
            raise ValueError('USA graphics resource entry signature mismatch')
        for lo,hi,expected in ((0x1e7fd0,0x1e8220,'ce0b9273fe9294c57b80a1fc698a25aadbcd67e3e2452ce48f630c87225673ba'),
                               (0x1fae30,0x1faeb0,'19049c76ab8ef6499d0b2b4d560a97b0b346d22665bcb5f628debd00ab5a8b51')):
            if hashlib.sha256(span(lo,hi)).hexdigest()!=expected:
                raise ValueError('USA camera vtable method signature mismatch')
    # Constructor sections are authoritative static entry metadata, independent
    # of whether the analysis export found every prologue. Never discover code
    # from runtime bytes or skip these application initializers.
    names_section=sections[header[13]]
    names=data[names_section[4]:names_section[4]+names_section[5]]
    targets=[]
    for section in sections:
        if names[section[0]:].split(b'\0')[0]!=b'.ctors':continue
        if section[5]%4 or section[4]+section[5]>len(data):raise ValueError('invalid constructor section')
        targets += [value[0] for value in struct.iter_unpack('<I',data[section[4]:section[4]+section[5]])]
    targets=sorted(set(targets));entries=[]
    for lo in targets:
        section=next((s for s in sections if s[2]&6==6 and s[3]<=lo<s[3]+s[5]),None)
        if lo&3 or section is None:raise ValueError('constructor target outside explicit code')
        hi=next((pc for pc in targets if pc>lo),section[3]+section[5]);hi=min(hi,section[3]+section[5])
        entries.append(dict(Name=f'module_initializer_{lo:08x}',Start=hex(lo),End=hex(hi)))
    if name=='mod001':
        entries.append(dict(Name='resource_upload_0029e240',Start='0x29e240',End='0x29e274'))
        # Explicit methods referenced by the camera's original main-image
        # vtable (guarded by game_entries). Tiny setters/getters were omitted
        # by the analysis export; preserve their original CPU bodies.
        bounds=[0x1e7fd0,0x1e7fe0,0x1e8000,0x1e8010,0x1e8020,0x1e8030,
                0x1e8040,0x1e8060,0x1e8080,0x1e80a0,0x1e80c0,0x1e80e0,0x1e8100]
        bounds += [0x1e81a0,0x1e81c0,0x1e81e0,0x1e8200,0x1e8220]
        for lo,hi in zip(bounds,bounds[1:]):
            if lo==0x1e8100:continue # existing exported methods cover this span
            entries.append(dict(Name=f'camera_method_{lo:08x}',Start=hex(lo),End=hex(hi)))
        for lo in range(0x1fae30,0x1faeb0,16):
            entries.append(dict(Name=f'camera_accessor_{lo:08x}',Start=hex(lo),End=hex(lo+16)))
    return entries


def static_call_targets(data,rows):
    _,segments,sections=elf_parts(data)
    spans=sorted((int(r['Start'],0),int(r['End'],0)) for r in rows
                 if any(s[2]&6==6 and s[3]<=int(r['Start'],0)<int(r['End'],0)<=s[3]+s[5] for s in sections))
    merged=[]
    for lo,hi in spans:
        if merged and lo<=merged[-1][1]:merged[-1][1]=max(hi,merged[-1][1])
        else:merged.append([lo,hi])
    targets=set()
    for lo,hi in merged:
        segment=next(s for s in segments if s[0]==1 and s[2]<=lo<hi<=s[2]+s[4])
        for pc in range(lo,hi,4):
            word=int.from_bytes(data[segment[1]+pc-segment[2]:segment[1]+pc-segment[2]+4],'little')
            if word>>26 in (2,3):targets.add(((pc+4)&0xf0000000)|((word&0x3ffffff)<<2))
    return targets


def literal_vtable_targets(callers,table_image):
    """Offline C++ method pointers, from literal vtable stores in admitted code.

    A table must have the original two-zero RTTI header and start with two
    method pointers into the input images' explicit code sections. Only a
    literal stored at a nonnegative aligned object offset is considered (base
    classes also have vptrs at nonzero offsets); arbitrary data words
    and runtime RAM never seed code. Constant tracking stops at control flow.
    """
    _,table_segments,_=elf_parts(table_image)
    code=[]
    for data,_ in callers:
        _,_,sections=elf_parts(data)
        code.extend((s[3],s[3]+s[5]) for s in sections if s[2]&6==6 and s[5])
    def executable(pointer):return pointer%4==0 and any(lo<=pointer<hi for lo,hi in code)
    def table_at(pointer):
        segment=next((s for s in table_segments if s[0]==1 and s[2]<=pointer< pointer+16<=s[2]+s[4]),None)
        if segment is None:return []
        at=segment[1]+pointer-segment[2];limit=segment[1]+segment[4]
        if table_image[at:at+8]!=bytes(8):return []
        first=struct.unpack_from('<2I',table_image,at+8)
        if not all(executable(value) for value in first):return []
        result=[]
        for offset in range(at+8,min(limit,at+8+1024),4):
            value=struct.unpack_from('<I',table_image,offset)[0]
            if value==0 and offset+8<=limit and executable(struct.unpack_from('<I',table_image,offset+4)[0]):
                continue # an unimplemented virtual slot inside this same table
            if not executable(value):break
            result.append(value)
        return result
    tables=set()
    for data,rows in callers:
        _,segments,sections=elf_parts(data)
        for row in rows:
            lo,hi=int(row['Start'],0),int(row['End'],0)
            if not any(s[2]&6==6 and s[3]<=lo<hi<=s[3]+s[5] for s in sections):continue
            segment=next(s for s in segments if s[0]==1 and s[2]<=lo<hi<=s[2]+s[4])
            constants={};clear_after=None
            for pc in range(lo,hi,4):
                if clear_after is not None and pc>=clear_after:
                    constants.clear();clear_after=None
                word=struct.unpack_from('<I',data,segment[1]+pc-segment[2])[0]
                op=word>>26;rs=(word>>21)&31;rt=(word>>16)&31;rd=(word>>11)&31;imm=word&65535
                if op==0x2b and imm<=0x1000 and imm%4==0 and rt in constants:
                    value,at=constants[rt]
                    if pc-at<=64 and table_at(value):tables.add(value)
                if op==15:
                    constants[rt]=(imm<<16,pc)
                elif op in (9,13) and rs in constants:
                    value,at=constants[rs]
                    value=(value+(imm-65536 if imm&32768 else imm))&0xffffffff if op==9 else value|imm
                    constants[rt]=(value,at)
                elif word==0 or op in (0x28,0x29,0x2b,0x1f,0x39,0x3f):
                    pass # stores/nop do not modify the tracked CPU constants
                elif op==0 and (word&63) not in (8,9):
                    constants.pop(rd,None)
                elif op==0x1c:
                    constants.pop(rd,None)
                elif op in (8,10,11,12,14,0x1e,0x20,0x21,0x23,0x24,0x25,0x27,0x37):
                    constants.pop(rt,None)
                elif op in (1,2,3,4,5,6,7,20,21,22,23) or (op==0 and word&63 in (8,9)) or (op in (16,17,18) and rs==8):
                    # The delay instruction still sees the caller's constants;
                    # destructors frequently store their vptr in a JAL delay.
                    if op==3:constants.pop(31,None)
                    elif op==0 and word&63==9:constants.pop(rd,None)
                    clear_after=pc+8
                else:
                    constants.clear() # no speculation across calls/branches/COP operations
                constants.pop(0,None)
    return {target for table in tables for target in table_at(table)}


def linked_call_entries(data,rows,targets,known=()):
    """Admit cross-image static references, bounded by the original code/map.

    This is offline metadata discovery, never a runtime instruction path.
    Existing covered interiors become labels; omitted leaves stop at return/tail
    transfer after all forward branches, or at the next declared function.
    """
    _,segments,sections=elf_parts(data)
    declared={int(r['Start'],0) for r in rows}|set(known)
    ranges=[(int(r['Start'],0),int(r['End'],0)) for r in rows]
    pending=set(targets)-declared;entries=[]
    while pending:
        start=min(pending);pending.remove(start)
        if start in declared:continue
        section=next((s for s in sections if s[2]&6==6 and s[3]<=start<s[3]+s[5]),None)
        if section is None or start&3:continue
        covered=next((hi for lo,hi in ranges if lo<start<hi),None)
        if covered is not None:end=min(start+4,covered)
        else:
            limit=min([section[3]+section[5]]+[pc for pc in declared|pending if start<pc<section[3]+section[5]])
            segment=next(s for s in segments if s[0]==1 and s[2]<=start<limit<=s[2]+s[4])
            end=limit;forward=start
            for pc in range(start,limit,4):
                at=segment[1]+pc-segment[2];word=int.from_bytes(data[at:at+4],'little');op=word>>26
                if op in (1,4,5,6,7,20,21,22,23) or (op in (16,17,18) and (word>>21)&31==8):
                    offset=word&0xffff;offset=offset-0x10000 if offset&0x8000 else offset
                    target=pc+4+offset*4
                    if start<=target<limit:forward=max(forward,target)
                terminal=(op==0 and word&63==8) or op==2
                if terminal and forward<=pc+4:
                    end=min(pc+8,limit);break
        row=dict(Name=f'linked_entry_{start:08x}',Start=hex(start),End=hex(end))
        entries.append(row);declared.add(start);ranges.append((start,end))
        pending |= static_call_targets(data,[row])-declared
        if len(entries)>4096:raise ValueError('static linked entry discovery exceeded its bound')
    return entries


def link_static_images(inputs,known):
    """Complete literal calls without mixing overlapping overlay code.

    Later overlays retain their own entry provenance. Alternating common-image
    call chains converge offline; there is no runtime code discovery.
    """
    counts={name:0 for name in inputs}
    for _ in range(32):
        references=set().union(*(static_call_targets(data,rows) for data,rows in inputs.values()))
        added=0
        for name,(data,rows) in inputs.items():
            targets=references if name in ('main','mod001') else static_call_targets(data,rows)
            entries=linked_call_entries(data,rows,targets,known.get(name,()))
            rows+=entries;counts[name]+=len(entries);added+=len(entries)
        if not added:return counts
        if sum(counts.values())>4096:raise ValueError('common-image linking exceeded its entry bound')
    raise ValueError('common-image linking did not converge')


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
    inputs={}
    for name in names:
        data=(args.build/'elf'/f'{name}.elf').read_bytes()
        with (args.build/'ghidra/export'/f'{name}_functions.csv').open(newline='') as stream:
            rows=list(csv.DictReader(stream))
        rows+=overlay_entries(name,data)
        if name=='main':
            from vp2_bozo_kernel import game_entries,bootstrap_tables
            rows+=game_entries(data)
            (args.out/'bootstrap.bin').write_bytes(bootstrap_tables(data))
        inputs[name]=(data,rows)
    vtable_references={}
    for name,(data,rows) in inputs.items():
        # Only this image's constructor stores nominate its virtual methods.
        # The common images supply legal base-class code ranges, without their
        # constructors nominating methods in an overlapping overlay.
        shared=[(other,[]) for key,(other,_) in inputs.items() if key in ('main','mod001') and key!=name]
        vtable_references[name]=literal_vtable_targets([(data,rows)]+shared,main_image)
    common_references=set().union(*vtable_references.values())
    for name,(data,rows) in inputs.items():
        references=common_references if name in ('main','mod001') else vtable_references[name]
        additions=linked_call_entries(data,rows,references,sdk_names(args.build/'sdk'/f'{name}.toml'))
        rows+=additions
        print(f'{name}: {len(additions)} static vtable entries exposed')
    linked=link_static_images(inputs,{name:sdk_names(args.build/'sdk'/f'{name}.toml') for name in names})
    for name,count in linked.items():print(f'{name}: {count} static call entries exposed')
    for name in names:
        data,rows=inputs[name]
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
