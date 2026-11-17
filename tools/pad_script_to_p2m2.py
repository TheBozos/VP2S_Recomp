"""Convert a Bozo route pad script into a PCSX2 input recording (.p2m2).

PCSX2 replays a .p2m2 from a VM reset with no window focus and pauses on its
last frame, so a reference frame can be reached without driving the keyboard.

Pad script: comma-separated `frame:BUTTON/duration` (duration in frames).
Usage: pad_script_to_p2m2.py SCRIPT OUT.p2m2 --frames N [--offset K]
       (--offset shifts every press K frames later; PCSX2 frame 0 is the reset)

Layout (PCSX2 pcsx2/Recording/InputRecordingFile.h, version 1): u8 version,
char[50] emulator, char[255] author, char[255] game, u32 total frames, u32 undo
count, u8 from-savestate, then per frame two ports of 18 bytes: two active-low
button bytes, right X/Y, left X/Y (neutral 0x7f), twelve pressure bytes.
"""
import argparse, struct, sys

# (byte, bit) of each button in the two active-low flag bytes, and its pressure index.
BUTTONS = {
    'LEFT': (0, 7, 7), 'DOWN': (0, 6, 9), 'RIGHT': (0, 5, 6), 'UP': (0, 4, 8),
    'START': (0, 3, None), 'R3': (0, 2, None), 'L3': (0, 1, None), 'SELECT': (0, 0, None),
    'SQUARE': (1, 7, 13), 'CROSS': (1, 6, 12), 'CIRCLE': (1, 5, 11), 'TRIANGLE': (1, 4, 10),
    'R1': (1, 3, 15), 'L1': (1, 2, 14), 'R2': (1, 1, 17), 'L2': (1, 0, 16),
}

def parse(text):
    presses = []
    for item in text.replace('\n', ',').split(','):
        item = item.strip()
        if not item or item.startswith('#'):
            continue
        frame, rest = item.split(':')
        names, _, duration = rest.partition('/')
        for name in names.split('+'):
            if name not in BUTTONS:
                raise ValueError(f'unknown button {name!r} in {item!r}')
            presses.append((int(frame), int(duration or 1), name))
    return presses

def idle():
    return bytearray([0xff, 0xff, 0x7f, 0x7f, 0x7f, 0x7f] + [0] * 12)

def build(presses, frames, offset=0, game='Valkyrie Profile 2 - Silmeria (USA)'):
    out = bytearray()
    out += bytes([1])
    for text, size in (('PCSX2-v2.8.2', 50), ('vp2s-route', 255), (game, 255)):
        out += text.encode()[:size - 1].ljust(size, b'\0')
    out += struct.pack('<IIB', frames, 0, 0)
    assert len(out) == 570
    pads = [idle() for _ in range(frames)]
    for start, duration, name in presses:
        byte, bit, pressure = BUTTONS[name]
        for f in range(start + offset, min(frames, start + offset + duration)):
            if f < 0:
                continue
            pads[f][byte] &= ~(1 << bit) & 0xff
            if pressure is not None:
                pads[f][pressure] = 0xff
    for pad in pads:
        out += pad + idle()  # port 1: no input
    return bytes(out)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('script'); ap.add_argument('out')
    ap.add_argument('--frames', type=int, required=True)
    ap.add_argument('--offset', type=int, default=0)
    a = ap.parse_args()
    data = build(parse(open(a.script, encoding='utf-8').read()), a.frames, a.offset)
    open(a.out, 'wb').write(data)
    print(f'{a.out}: {a.frames} frames, {len(data)} bytes')

if __name__ == '__main__':
    sys.exit(main())
