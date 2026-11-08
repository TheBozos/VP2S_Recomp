"""Offline fixed particle-template materials -> named native draw semantics.

This converter reads the shipped template layout, never a live transport chain.
No console register words or addresses are retained in the output.
"""
import argparse,json,struct
from pathlib import Path

MARKER=struct.pack('<I',0x6c0902e0)

def convert(data):
    records=[];cursor=0
    while True:
        marker=data.find(MARKER,cursor)
        if marker<0:break
        cursor=marker+4;at=marker-0x1c
        if at<0 or at&15 or at+0xc0>len(data):continue
        n=data[at]*data[at+1]
        if not 1<=n<=16 or at+0xc0+80*n>len(data):continue
        word=lambda offset:struct.unpack_from('<I',data,at+offset)[0]
        wide=lambda offset:struct.unpack_from('<Q',data,at+offset)[0]
        if word(0xbc)!=(0x6c0002e9|((5*n)<<16)):continue
        if any(wide(offset)!=reg for offset,reg in ((0x28,5),(0x48,6),(0x58,6),(0x78,0x42),(0x88,0x42),(0x98,0x42),(0xa8,0x42))):continue
        texture=wide(0x40);wrap=wide(0x20)
        fields=[at,n,0xc0+80*n,(texture>>34)&1,(texture>>35)&3,wrap&3,(wrap>>2)&3]
        for offset in (0x70,0x80,0x90,0xa0):
            blend=wide(offset);factors=[blend&3,(blend>>2)&3,(blend>>4)&3,(blend>>6)&3,(blend>>32)&255]
            if any(v>2 for v in factors[:4]):break
            fields+=factors
        if len(fields)!=27:continue
        records.append(struct.pack('<27I',*fields))
    return struct.pack('<4sI',b'VPP1',len(records))+b''.join(records)

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--assets',type=Path,required=True);p.add_argument('--out',type=Path,required=True);a=p.parse_args()
    a.out.mkdir(parents=True,exist_ok=True);index=a.out/'conversion-index.json'
    previous=json.loads(index.read_text()) if index.is_file() else {};updated={};files=records=0
    tool_stamp=Path(__file__).stat().st_mtime_ns
    for source in sorted(a.assets.rglob('*')):
        if not source.is_file():continue
        stat=source.stat();identity=source.relative_to(a.assets).as_posix();stamp=[stat.st_size,stat.st_mtime_ns,tool_stamp]
        old=previous.get(identity)
        if old and old[:3]==stamp and (not old[3] or (a.out/(source.stem+'.pmat')).is_file()):count=old[3]
        else:
            output=convert(source.read_bytes());count=struct.unpack_from('<I',output,4)[0]
            target=a.out/(source.stem+'.pmat')
            if count or target.is_file():target.write_bytes(output)
        updated[identity]=stamp+[count];files+=1;records+=count
    index.write_text(json.dumps(updated,separators=(',',':')))
    print(f'{records} native particle material templates across {files} decoded files')

if __name__=='__main__':main()
