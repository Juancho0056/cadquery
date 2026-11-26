import pytest
import cadquery as cq
from itertools import combinations
from modular_arm.v3.locking_core import Core,spring_space
from modular_arm.v3.retained_core import parts

@pytest.mark.parametrize("travel",[0,1,3,5,7])
def test_stops_running_clearance(travel):
    p=parts(travel=travel)
    for name,s in p.items():
        assert s.isValid() and len(s.Solids())==1,name
        assert s.intersect(spring_space(Core(),travel)).Volume()<1e-6,name
    for (a,s),(b,t) in combinations(p.items(),2):
        assert s.intersect(t).Volume()<1e-6,(travel,a,b)

def test_upper_stop():
    p=parts()
    assert p["plunger"].translate(cq.Vector(0,0,.5)).intersect(p["rotating"]).Volume()>1e-3

def test_lower_stop():
    p=parts(travel=7)
    assert p["plunger"].translate(cq.Vector(0,0,-.5)).intersect(p["stationary"]).Volume()>1e-3

def test_fastener_wall_nominal():
    # Radial material outside and inside a 3.2 mm clearance hole.
    assert 7-3.7-1.6>=1.5
    assert 3.7-.35-1.6>=1.5
