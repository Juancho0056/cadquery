import pytest
import cadquery as cq
from itertools import combinations
from modular_arm.v3.retained_core import parts

@pytest.mark.parametrize("angle,travel",[(0,0),(45,0),(22.5,7),(90,7)])
def test_retained_poses(angle,travel):
    p=parts(angle=angle,travel=travel)
    for n,s in p.items():
        assert s.isValid() and len(s.Solids())==1,n
    for (a,s),(b,t) in combinations(p.items(),2):
        assert s.intersect(t).Volume()<1e-6,(a,b)

def test_axial_capture():
    p=parts(travel=7)
    rotor=p["rotating"]
    assert rotor.translate(cq.Vector(0,0,1)).intersect(p["keeper"]).Volume()>1e-3
    assert rotor.translate(cq.Vector(0,0,-1)).intersect(p["stationary"]).Volume()>1e-3

def test_radial_capture():
    p=parts(travel=7)
    assert p["rotating"].translate(cq.Vector(1,0,0)).intersect(p["stationary"]).Volume()>1e-3
