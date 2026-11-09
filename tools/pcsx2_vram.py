"""PSMCT32 reads of the VRAM in a PCSX2 GS dump's state (509-byte header + 4 MB), with the GS block/column swizzle."""
import struct
from PIL import Image

BLOCK32 = [[0, 1, 4, 5, 16, 17, 20, 21], [2, 3, 6, 7, 18, 19, 22, 23],
           [8, 9, 12, 13, 24, 25, 28, 29], [10, 11, 14, 15, 26, 27, 30, 31]]
COLUMN32 = [[0, 1, 4, 5, 8, 9, 12, 13], [2, 3, 6, 7, 10, 11, 14, 15], [16, 17, 20, 21, 24, 25, 28, 29],
            [18, 19, 22, 23, 26, 27, 30, 31], [32, 33, 36, 37, 40, 41, 44, 45], [34, 35, 38, 39, 42, 43, 46, 47],
            [48, 49, 52, 53, 56, 57, 60, 61], [50, 51, 54, 55, 58, 59, 62, 63]]


def vram(state):
    return state[509:509 + 4 * 1024 * 1024]


def word32(vm, bp, bw, x, y):
    """bp in 256-byte blocks, bw in 64-pixel units."""
    page = (y // 32) * bw + x // 64
    block = bp + page * 32 + BLOCK32[(y % 32) // 8][(x % 64) // 8]
    at = (block * 64 + COLUMN32[y % 8][x % 8]) * 4
    return struct.unpack_from('<I', vm, at % len(vm))[0]


def image32(vm, bp, bw, width, height):
    """RGBA image (alpha as stored, 0x80 = 1.0)."""
    im = Image.new('RGBA', (width, height))
    px = im.load()
    for y in range(height):
        for x in range(width):
            w = word32(vm, bp, bw, x, y)
            px[x, y] = (w & 255, (w >> 8) & 255, (w >> 16) & 255, (w >> 24) & 255)
    return im
