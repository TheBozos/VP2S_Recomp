"""Validate USA matrix helper and emit its original algorithm for offline checking.

The native game uses a typed C++ matrix operation, never these reference files.
"""
import argparse
import importlib.util
import struct
import sys
from pathlib import Path


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--elf',type=Path,required=True)
    ap.add_argument('--out',type=Path,required=True)
    args=ap.parse_args()
    data=args.elf.read_bytes()
    code=data[0x12b4a8+0x250:0x12b4a8+0x2d0]
    expected=[]
    for column in range(1,5):
        expected.extend([0x01e029bc|(column<<16),0x01e030bd|(column<<16),
                         0x01e038be|(column<<16)|(0x40000000 if column==4 else 0),
                         0x01e0400b|(column<<16)|(column<<6)])
    pairs=list(struct.iter_unpack('<II',code))
    if [a for a,b in pairs]!=[0x8000033c]*16 or [b for a,b in pairs]!=expected:
        raise ValueError('Unexpected USA matrix product; refusing native helper build')
    sys.dont_write_bytecode=True
    path=Path(__file__).resolve().parent.parent/'external/PS2Recomp/tools/vu1_lift.py'
    spec=importlib.util.spec_from_file_location('matrix_lifter',path)
    lift=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(lift)
    source,_=lift.generate(code,dict(hash=lift.code_hash(code),pairs=16,
                            function='matrix_reference',entries=[0],targets=[],unit='VU0'))
    args.out.mkdir(parents=True,exist_ok=True)
    path=args.out/'reference.cpp'
    source=source.encode()
    if not path.exists() or path.read_bytes()!=source: path.write_bytes(source)
    print('Native matrix: validated USA product; original reference emitted offline')


if __name__=='__main__': main()
