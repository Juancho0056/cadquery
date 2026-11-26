import pytest
from itertools import combinations
from modular_arm.v4.rigid_ends import parts,keyed_clevis,keyed_tongue
from modular_arm.v4.reference_ends import box

@pytest.mark.parametrize("angle,travel",[(0,0),(45,0),(-45,0),(90,0),(-90,0),(0,7),(30,7),(-30,7),(90,7),(-90,7)])
def test_candidate_poses(angle,travel):
    p=parts(angle,travel)
    for n,s in p.items():
        assert s.isValid() and len(s.Solids())==1,n
    for (a,s),(b,t) in combinations(p.items(),2):
        assert s.intersect(t).Volume()<1e-6,(angle,a,b)

def test_keyed_clevis_nominal_gauge():
    # Simplified rectangular tongue envelope, not the original STL.
    gauge=box(-5.75,-13.95,-11,11.5,27.9,21)
    s=keyed_clevis()
    assert s.intersect(gauge).Volume()<1e-6
    assert s.intersect(gauge.rotate((0,0,0),(1,0,0),10)).Volume()>1e-3

def test_keyed_blade_width():
    assert keyed_tongue().BoundingBox().xlen==pytest.approx(10.6)
    assert keyed_tongue().BoundingBox().ylen==pytest.approx(23.)
