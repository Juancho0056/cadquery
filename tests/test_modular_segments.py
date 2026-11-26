from itertools import combinations
import pytest
from modular_arm.assembly.common_selector import CommonSelectorConfig,common_selector_parts
from modular_arm.assembly.arm_interfaces import interface_solids
from modular_arm.assembly.modular_segments import stop_solids,extension_parts

@pytest.fixture(scope="module")
def c():
    return CommonSelectorConfig()

def test_stop_geometry(c):
    _,arm=interface_solids(c)
    for sign,stop in stop_solids(c).items():
        for deg in range(-30,31,5):
            assert arm.rotate((0,0,0),(0,0,1),deg).intersect(stop).Volume()<1e-6,(sign,deg)
        assert arm.rotate((0,0,0),(0,0,1),sign*31).intersect(stop).Volume()>1e-3

def test_extension_solids(c):
    p=extension_parts(c)
    assert len(p)==10
    for name,s in p.items():
        assert s.isValid() and len(s.Solids())==1,name
    for (a,p),(b,q) in combinations(p.items(),2):
        assert p.intersect(q).Volume()<1e-6,(a,b)

@pytest.mark.parametrize("angle",[-30.,0.,30.])
def test_complete_modular_poses(c,angle):
    p=common_selector_parts(c,include_arms=True,include_stops=True,include_extensions=True,rotor_angle=angle)
    for name,s in p.items():
        assert s.isValid() and len(s.Solids())==1,name
    for (a,p),(b,q) in combinations(p.items(),2):
        assert p.intersect(q).Volume()<1e-6,(angle,a,b)
