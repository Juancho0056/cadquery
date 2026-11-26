import pytest
from itertools import combinations
from modular_arm.v2.elbow import Elbow,elbow_parts
from modular_arm.v2.connector import plug
import cadquery as cq

@pytest.mark.parametrize("angle",[-90,-60,-30,0,30,60,90])
def test_indexed_elbow(angle):
    parts=elbow_parts(angle=angle)
    for name,p in parts.items():
        assert p.isValid() and len(p.Solids())==1,name
    for (a,p),(b,q) in combinations(parts.items(),2):
        assert p.intersect(q).Volume()<1e-6,(angle,a,b)

def test_lock_blocks_rotation():
    p=elbow_parts(angle=0)
    assert p["moving"].rotate((0,0,0),(0,0,1),3).intersect(p["index_pin"]).Volume()>1e-3

def test_unlocked_pose():
    assert "index_pin" not in elbow_parts(angle=17,locked=False)

def test_nonindexed_lock():
    with pytest.raises(ValueError):
        elbow_parts(angle=17)
