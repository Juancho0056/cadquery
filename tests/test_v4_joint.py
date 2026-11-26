import pytest
import cadquery as cq
from itertools import combinations
from modular_arm.v4.joint import parts,spring_space,DIAMETER,button,moving

@pytest.mark.parametrize("angle,travel",[(a,0.) for a in range(-180,181,45)]+[(a,7.) for a in range(-180,181,15)]+[(0,t) for t in (1,3,5,6)])
def test_v4_poses(angle,travel):
    p=parts(angle,travel)
    for n,s in p.items():
        assert s.isValid() and len(s.Solids())==1,n
        assert s.intersect(spring_space(travel)).Volume()<1e-6,n
    for (a,s),(b,t) in combinations(p.items(),2):
        assert s.intersect(t).Volume()<1e-6,(angle,travel,a,b)

def test_top_loading():
    p=parts(hardware=False)
    for dz in range(0,41,2):
        for n in ("arm_a","button"):
            assert p["arm_b"].translate(cq.Vector(0,0,dz)).intersect(p[n]).Volume()<1e-6,(dz,n)
        assert p["button"].translate(cq.Vector(0,0,dz)).intersect(p["arm_a"]).Volume()<1e-6

def test_lock():
    assert moving().rotate((0,0,0),(0,0,1),10).intersect(button()).Volume()>1e-3

def test_axial_retention():
    p=parts(travel=7)
    for dz in (-1.,1.):
        moved=[s.translate(cq.Vector(0,0,dz)) for n,s in p.items() if n.startswith("retainer_")]
        assert any(s.intersect(p["arm_a"]).Volume()>1e-3 for s in moved)

def test_button_stops():
    p=parts()
    assert p["button"].translate(cq.Vector(0,0,.5)).intersect(p["arm_b"]).Volume()>1e-3
    p=parts(travel=7)
    assert p["button"].translate(cq.Vector(0,0,-.5)).intersect(p["arm_a"]).Volume()>1e-3
