"""Offline VPM3 -> semantic native mesh records. Never used by the game runner."""
import argparse
import struct
from pathlib import Path


def convert(data):
    at = 0
    def take(fmt):
        nonlocal at
        size=struct.calcsize('<'+fmt)
        if at+size>len(data):raise ValueError('truncated mesh pack')
        result=struct.unpack_from('<'+fmt,data,at);at+=size
        return result
    if take('4s')[0]!=b'VPM3':raise ValueError('VPM3 required')
    count,=take('I');out=bytearray(struct.pack('<4sI',b'VPN1',count))
    for _ in range(count):
        offset,nv,ns,flags,group,*rest=take('4Ii8i3fI20I')
        routines=rest[:4];material=rest[11]
        # Named semantic fields: blend enabled, texture enabled, texture alpha,
        # texture function, lighting mode, backface suppression, fog enabled,
        # then four independent native blend factors. No transport words remain.
        if material:
            m=take('16I');alpha,mode,_,tex,prim,cull=m[:6]
            semantics=[bool(prim&64),bool(prim&16),bool(tex&4),(tex>>3)&3,
                       mode,cull==0,bool(prim&32),alpha&3,(alpha>>2)&3,(alpha>>4)&3,(alpha>>6)&3]
        else:semantics=[0]*11
        # Algorithm identities are asset metadata. Unsupported algorithms are
        # rejected by the native builder when reached, never interpreted.
        out+=struct.pack('<4I4i12I',offset,nv,ns,group&0xffffffff,*routines,material,*semantics)
        strips=take(f'{ns}H');out+=struct.pack(f'<{ns}H',*strips)
        at=(at+3)&~3
        if sum(strips)!=nv:raise ValueError('strip vertices differ')
        out+=struct.pack(f'<{nv*10}I',*take(f'{nv*10}I'))
        if 4 in routines:out+=struct.pack(f'<{nv*4}I',*take(f'{nv*4}I'))
        if 48 in routines:out+=struct.pack('<i',*take('i'))
        size,=take('I');sources=take(f'{nv*2}I')
        if any(s!=0xffffffff and s+12>size for s in sources):raise ValueError('vertex source outside mesh')
        out+=struct.pack(f'<I{nv*2}I',size,*sources)
    if at!=len(data):raise ValueError('trailing mesh pack bytes')
    return bytes(out)


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--source',type=Path,required=True);ap.add_argument('--out',type=Path,required=True)
    args=ap.parse_args();args.out.mkdir(parents=True,exist_ok=True)
    count=0
    for source in sorted(args.source.glob('*.msh')):
        target=args.out/(source.stem+'.mesh')
        if target.is_file() and target.stat().st_mtime_ns>=max(source.stat().st_mtime_ns,Path(__file__).stat().st_mtime_ns):
            count+=1;continue
        converted=convert(source.read_bytes())
        if not target.is_file() or target.read_bytes()!=converted:target.write_bytes(converted)
        else:target.touch()
        count+=1
    if not count:raise SystemExit('no offline mesh packs')
    print(f'{count} semantic native mesh packs')


if __name__=='__main__':main()
