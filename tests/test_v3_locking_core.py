import pytest
from itertools import combinations
from modular_arm.v3.locking_core import Core,parts,profile,plunger,spring_space,effective_engagement,spring_force_estimate,rotating

@pytest.mark.parametrize("travel",[0.,1.,3.,5.,6.,7.])
def test_axial_travel(travel):
    c=Core()
    p=parts(c,travel=travel)
    for name,s in p.items():
        assert s.isValid() and len(s.Solids())==1,name
        assert s.intersect(spring_space(c,travel)).Volume()<1e-6,name
    for (a,s),(b,t) in combinations(p.items(),2):
        assert s.intersect(t).Volume()<1e-6,(travel,a,b)

@pytest.mark.parametrize("angle",[0.,5.,15.,22.5,30.,45.,90.,135.,180.])
def test_full_release(angle):
    p=parts(angle=angle,travel=7.)
    assert p["rotating"].intersect(p["plunger"]).Volume()<1e-6
    assert p["rotating"].intersect(p["stationary"]).Volume()<1e-6

def test_positive_lock():
    c=Core()
    assert rotating(c).rotate((0,0,0),(0,0,1),10).intersect(plunger(c)).Volume()>1e-3
    assert effective_engagement(c)==6.
    assert effective_engagement(c,7.)==0.

def test_return_screening():
    c=Core()
    assert spring_force_estimate(c,0)>0
    assert spring_force_estimate(c,7)>spring_force_estimate(c,0)
    assert 13-7>c.spring_solid

def test_minimum_geometry():
    with pytest.raises(ValueError):
        Core(tip_radius=16.)
    with pytest.raises(ValueError):
        Core(stroke=5.)
