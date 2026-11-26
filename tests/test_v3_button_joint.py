import pytest
from itertools import combinations
from modular_arm.v3.button_joint import Joint,parts,spring_envelope

@pytest.mark.parametrize("angle",[-90,-45,0,45,90])
def test_engaged(angle):
    p=parts(angle=angle)
    for n,s in p.items():
        assert s.isValid() and len(s.Solids())==1,n
    for (a,s),(b,t) in combinations(p.items(),2):
        assert s.intersect(t).Volume()<1e-6,(a,b)
    assert p["rotor"].rotate((0,0,0),(0,0,1),5).intersect(p["button_dog"]).Volume()>1e-3

@pytest.mark.parametrize("angle",range(-90,91,10))
def test_pressed(angle):
    p=parts(angle=angle,pressed=True)
    for (a,s),(b,t) in combinations(p.items(),2):
        assert s.intersect(t).Volume()<1e-6,(angle,a,b)
    spring=spring_envelope(Joint(),True)
    for n,s in p.items():
        assert spring.intersect(s).Volume()<1e-6,n

def test_nonindexed():
    with pytest.raises(ValueError):
        parts(angle=17)

def test_minimum_dog_section():
    c=Joint()
    assert c.tooth_width>=5
    assert c.tooth_radius-c.core_radius>=4
    assert 360//c.pitch==8
    with pytest.raises(ValueError):
        Joint(tooth_width=3.)
