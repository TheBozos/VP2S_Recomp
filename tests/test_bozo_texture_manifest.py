"""Asset identity selection is offline and rejects missing/conflicting copies."""
import sys
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from vp2_bozo_texture_manifest import manifest
class TextureManifest(unittest.TestCase):
    def test_shared_identical_copies(self):
        index=[dict(file='2\\one.mod',at=20,tag=4,index=9),dict(file='3\\two.bin',at=40,tag=4,index=9)]
        self.assertEqual(manifest(index,{('one',20):'c_a.tex',('two',40):'c_a.tex'}),{(4,9):'c_a.tex'})
    def test_conflicting_copies_require_provenance(self):
        index=[dict(file='one.mod',at=20,tag=4,index=9),dict(file='two.bin',at=40,tag=4,index=9)]
        self.assertEqual(manifest(index,{('one',20):'c_a.tex',('two',40):'c_b.tex'}),{})
    def test_missing_copy_does_not_admit_last_known(self):
        index=[dict(file='one.mod',at=20,tag=4,index=9),dict(file='two.bin',at=40,tag=4,index=9)]
        self.assertEqual(manifest(index,{('one',20):'c_a.tex'}),{})
    def test_key_and_index_are_distinct(self):
        index=[dict(file='one.mod',at=i,tag=4,index=i) for i in (9,10)]
        self.assertEqual(len(manifest(index,{('one',9):'c_a.tex',('one',10):'c_b.tex'})),2)
if __name__=='__main__':unittest.main()
