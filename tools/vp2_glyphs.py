"""Convert mcps font glyphs OFFLINE to immutable native RGBA packs.

Runtime selection uses the game's decompressed-file provenance. No runtime
glyph nibble, palette or upload-payload decoder is needed.
"""
import argparse
from dataclasses import dataclass
import hashlib
from pathlib import Path
import struct

MAGIC=b'VPGF0001'
HEADER=struct.Struct('<8s9I')

@dataclass(frozen=True)
class Font:
    offset:int
    count:int
    stride:int
    width:int
    height:int
    border:int
    line_height:int

def inspect_font(data):
    if len(data)<0x80 or data[:8]!=b'mcps2lib':raise ValueError('not an admitted mcps font')
    size,=struct.unpack_from('<I',data,0x20)
    offset,count,_,stride,line_height,width,height,border=struct.unpack_from('<8I',data,0x30)
    if not count:return None
    if not (0<size<=len(data) and 0<width<=256 and 0<height<=256 and 0<count<=16384 and
            0<line_height<=height and border<=16 and stride>=(width*height+1)//2 and
            offset>=0x80 and offset+count*stride<=size):
        raise ValueError('invalid mcps glyph bounds/metrics')
    return Font(offset,count,stride,width,height,border,line_height)

def convert(data,palette):
    font=inspect_font(data)
    if font is None:raise ValueError('mcps contains no glyphs')
    if len(palette)!=256:raise ValueError('offline palette needs 256 native entries')
    cell_width=1<<(font.width-1).bit_length();cell_height=1<<(font.height-1).bit_length()
    size=font.count*cell_width*cell_height*4*2
    if size>128*1024*1024:raise ValueError('converted font exceeds native pack bound')
    output=bytearray(HEADER.pack(MAGIC,font.width,font.height,font.count,font.stride,
                                 cell_width,cell_height,font.border,font.line_height,2))
    for csa in (1,2):
        for glyph in range(font.count):
            image=bytearray(cell_width*cell_height*4)
            base=font.offset+glyph*font.stride
            for y in range(font.height):
                for x in range(font.width):
                    k=y*font.width+x;packed=data[base+k//2]
                    index=(packed>>4) if k&1 else packed&15
                    # Keep 0x80-scale alpha; presentation expands it once.
                    struct.pack_into('<I',image,(y*cell_width+x)*4,palette[csa*16+index])
            output.extend(image)
    return bytes(output)

def glyph_palette(directory,manifest):
    lines=manifest.read_text().splitlines()
    if not lines or lines[0]!='VPNT1':raise ValueError('native texture manifest version mismatch')
    names=[line.split()[2] for line in lines[1:] if line.split()[:2]==['00494853','41']]
    if len(names)!=1:raise ValueError('unambiguous offline glyph palette required')
    name=names[0]
    if Path(name).name!=name:raise ValueError('offline texture filename must be local')
    data=(directory/name).read_bytes()
    if len(data)<12:raise ValueError('truncated offline glyph palette')
    width,height,fmt=struct.unpack_from('<3I',data)
    if not (0<width<=1024 and 0<height<=1024 and fmt in (0,0x13,0x14)):
        raise ValueError('unsupported offline palette texture')
    at=12+width*height*(4 if fmt==0 else 1)
    if len(data)!=at+1024:raise ValueError('offline palette size mismatch')
    return struct.unpack_from('<256I',data,at)

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--assets',type=Path,required=True);p.add_argument('--textures',type=Path,required=True)
    p.add_argument('--texture-manifest',type=Path,required=True);p.add_argument('--out',type=Path,required=True)
    a=p.parse_args();palette=glyph_palette(a.textures,a.texture_manifest)
    a.out.mkdir(parents=True,exist_ok=True);entries={};packs={};skipped=0;fonts={}
    # Standalone .mcps bundles, and mcps fonts embedded at 16-byte boundaries
    # inside other decompressed files (e.g. a model container). The key is the
    # glyph pixels' offset within the decompressed part.
    def embedded():
        for source in sorted(a.assets.rglob('*_*.*')):
            if not source.is_file():continue
            whole=source.read_bytes()
            if source.suffix=='.mcps':yield source,whole,0;continue
            at=whole.find(b'mcps2lib')
            while at>=0:
                if at%16==0 and at+0x80<=len(whole):
                    size,=struct.unpack_from('<I',whole,at+0x20)
                    if 0<size<=len(whole)-at:yield source,whole[at:at+size],at
                at=whole.find(b'mcps2lib',at+1)
    for source,data,base in embedded():
        font=inspect_font(data)
        if font is None:skipped+=1;continue
        stem,part=source.stem.rsplit('_',1);key=(int(stem,16),int(part),base+font.offset)
        # Deduplicate source glyphs before conversion; repeated text bundles
        # generally share the same font. Different metrics/palette stay distinct.
        identity=hashlib.sha256(repr(font).encode()+data[font.offset:font.offset+font.count*font.stride]+struct.pack('<256I',*palette)).digest()
        if identity not in fonts:
            packed=convert(data,palette);name='g_'+hashlib.sha1(packed).hexdigest()+'.vpg'
            packs.setdefault(name,packed);fonts[identity]=name
        name=fonts[identity]
        if key in entries and entries[key]!=name:raise ValueError('ambiguous font provenance')
        entries[key]=name
    for name,data in packs.items():
        path=a.out/name
        if not path.exists() or path.read_bytes()!=data:path.write_bytes(data)
    manifest='VPGFM1\n'+''.join(f'{disc:x} {part:x} {offset:x} {name}\n' for (disc,part,offset),name in sorted(entries.items()))
    (a.out/'manifest.txt').write_text(manifest)
    print(f'{len(entries)} mcps font origins; {len(packs)} unique native RGBA packs; {skipped} fontless bundles; {sum(map(len,packs.values()))} converted bytes')

if __name__=='__main__':main()
