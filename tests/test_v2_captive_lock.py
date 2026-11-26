import pytest
import cadquery as cq
from itertools import combinations
from modular_arm.v2.elbow import Elbow,elbow_parts,captive_lock

@pytest.mark.parametrize("angle,locked",[(0,True),(60,True),(-90,True),(17,False),(-45,False),(90,False)])
def test_captive_lock_poses(angle,locked):
    p=elbow_parts(angle=angle,locked=locked,captive=True)
    for name,s in p.items():
        assert s.isValid() and len(s.Solids())==1,name
    for (a,s),(b,t) in combinations(p.items(),2):
        assert s.intersect(t).Volume()<1e-6,(angle,a,b)

def test_capture_shoulder():
    c=Elbow()
    p=captive_lock(c,False)
    assert p["index_pin"].translate(cq.Vector(0,0,1.)).intersect(p["lock_cap"]).Volume()>1e-3

def test_lock_engagement():
    p=elbow_parts(angle=0,captive=True)
    assert p["moving"].rotate((0,0,0),(0,0,1),3).intersect(p["index_pin"]).Volume()>1e-3
