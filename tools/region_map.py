"""Map addresses of the European build (SLES-54645) to the American one (SLUS-21452).

    python tools/region_map.py [--eu build] [--us build_usa] <address or file> ...

Addresses are looked up in every wrapped ELF of the European tree (build/elf: main.elf, modNNN.elf) that holds them,
then found in the American ELF of the same name. Code: the instructions from the address on, with the fields a
relocation or a moved global changes masked (the immediate of non-register instructions, the target of j / jal), as
long a window as it takes to match once. Data (an address with no code there or no code match): the code that loads
it (lui / addiu or lui / load-store pairs) mapped as code, the address read back from the American instructions.

A file argument: every 0x00XXXXXX constant in it is mapped. Prints "eu -> us (module, how)".
"""
from __future__ import annotations

import argparse
import re
import struct
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent


class Elf:
    def __init__(self, path: Path):
        self.name = path.stem
        data = path.read_bytes()
        phoff = struct.unpack_from("<I", data, 0x1C)[0]
        count = struct.unpack_from("<H", data, 0x2C)[0]
        self.segments = []  # (vaddr, bytes)
        self.ranges = []  # (vaddr, memsz): bss included
        for i in range(count):
            kind, off, va, _, filesz, memsz = struct.unpack_from("<6I", data, phoff + 32 * i)[:6]
            if kind == 1 and filesz:
                self.segments.append((va, data[off:off + filesz]))
            if kind == 1 and memsz:
                self.ranges.append((va, memsz))

    def holds(self, address: int) -> bool:
        return any(va <= address < va + size for va, size in self.ranges)

    def word(self, address: int) -> int | None:
        for va, b in self.segments:
            if va <= address and address + 4 <= va + len(b):
                return struct.unpack_from("<I", b, address - va)[0]
        return None

    def words(self):
        for va, b in self.segments:
            for k in range(0, len(b) - 3, 4):
                yield va + k, struct.unpack_from("<I", b, k)[0]


def mask(word: int) -> int:
    op = word >> 26
    if op == 0:
        return word
    if op in (2, 3):
        return word & 0xFC000000
    return word & 0xFFFF0000


def looks_like_code(elf: Elf, address: int) -> bool:
    w = [elf.word(address + 4 * k) for k in range(4)]
    return all(x is not None for x in w) and any(x != 0 for x in w) and not all((x >> 24) in (0x3F, 0x3E, 0xBF, 0xBE, 0x00) and x & 0xFFFF == 0 for x in w)


def map_code(eu: Elf, us: Elf, address: int, us_index: dict) -> int | None:
    for length in (6, 10, 16, 24, 40):
        pattern = []
        for k in range(length):
            w = eu.word(address + 4 * k)
            if w is None:
                break
            pattern.append(mask(w))
        if len(pattern) < 4:
            return None
        hits = [a for a in us_index.get(tuple(pattern[:2]), []) if all(
            (w := us.word(a + 4 * k)) is not None and mask(w) == pattern[k] for k in range(len(pattern)))]
        if len(hits) == 1:
            return hits[0]
        if not hits:
            return None
        # several: the one nearest the European address
        if length == 40:
            return min(hits, key=lambda a: abs(a - address))
    return None


def map_data(eu: Elf, us: Elf, address: int, us_index: dict) -> int | None:
    """The data address from the code that builds it: lui hi(address) then an instruction adding lo(address)."""
    hi, lo = (address + 0x8000) >> 16, address & 0xFFFF
    for at, w in eu.words():
        if w >> 26 != 0x0F or (w & 0xFFFF) != hi:
            continue
        reg = (w >> 16) & 31
        for k in range(1, 8):
            n = eu.word(at + 4 * k)
            if n is None or (n >> 26) == 0 or ((n >> 21) & 31) != reg or (n & 0xFFFF) != lo:
                continue
            start = at - 8
            mapped = map_code(eu, us, start, us_index)
            if mapped is None:
                continue
            uw, un = us.word(mapped + 8), us.word(mapped + 8 + 4 * k)
            if uw is None or un is None or uw >> 26 != 0x0F:
                continue
            return ((uw & 0xFFFF) << 16) + struct.unpack("<h", struct.pack("<H", un & 0xFFFF))[0]
    return None


def index(elf: Elf) -> dict:
    out: dict = {}
    prev = None
    for at, w in elf.words():
        if prev is not None:
            out.setdefault((mask(prev[1]), mask(w)), []).append(prev[0])
        prev = (at, w)
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--eu", default="build")
    ap.add_argument("--us", default="build_usa")
    ap.add_argument("items", nargs="+")
    args = ap.parse_args()
    addresses: list[int] = []
    for item in args.items:
        if Path(item).is_file():
            addresses += [int(m, 16) for m in re.findall(r"0x(?:00)?([1-5][0-9A-Fa-f]{5})(?![0-9A-Fa-f])", Path(item).read_text(errors="replace"))]
        else:
            addresses.append(int(item, 16))
    addresses = sorted(set(addresses))
    eus = {p.stem: Elf(p) for p in sorted((ROOT / args.eu / "elf").glob("*.elf"))}
    uss = {p.stem: Elf(p) for p in sorted((ROOT / args.us / "elf").glob("*.elf"))}
    indexes: dict = {}
    for address in addresses:
        results = []
        for name, eu in eus.items():
            if not eu.holds(address) or name not in uss:
                continue
            us = uss[name]
            idx = indexes.setdefault(name, index(us))
            how, mapped = "code", map_code(eu, us, address, idx) if looks_like_code(eu, address) else None
            if mapped is None:
                how, mapped = "data", map_data(eu, us, address, idx)
            if mapped is not None:
                results.append(f"{mapped:08x} ({name}, {how})")
        if not results:
            # Data no code of its own ELF builds (a main class's vtable set up by an overlay): any ELF's code.
            for name, eu in eus.items():
                if name not in uss:
                    continue
                mapped = map_data(eu, uss[name], address, indexes.setdefault(name, index(uss[name])))
                if mapped is not None:
                    results.append(f"{mapped:08x} (from {name} code, data)")
                    break
        print(f"{address:08x} -> " + ("; ".join(dict.fromkeys(results)) if results else "?"))
        sys.stdout.flush()


if __name__ == "__main__":
    main()
