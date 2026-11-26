import pytest
from itertools import combinations
from modular_arm.mechanisms.selector_contract import Mode
from modular_arm.mechanisms.selector_slider import SelectorConfig,selector_position,cam_rises,selector_bench,make_spring_coupon

@pytest.mark.parametrize("mode,expected",[(Mode.A,(8,0)),(Mode.B,(0,0)),(Mode.C,(0,8))])
def test_nominal_cam_command(mode,expected):
    c=SelectorConfig()
    assert cam_rises(c,selector_position(c,mode))==expected

def test_nominal_transition_never_commands_both():
    c=SelectorConfig()
    for i in range(101):
        x=-c.stroke_mm+2*c.stroke_mm*i/100
        a,b=cam_rises(c,x)
        assert min(a,b)==0

@pytest.mark.parametrize("mode",list(Mode))
def test_bench_solids(mode):
    parts=selector_bench(mode=mode)
    for name,p in parts.items():
        assert p.isValid() and len(p.Solids())==1,name
    for (a,p),(b,q) in combinations(parts.items(),2):
        assert p.intersect(q).Volume()<1e-6,(a,b)

def test_spring_coupon_dimensions():
    c=SelectorConfig()
    p=make_spring_coupon(c)
    assert p.BoundingBox().xlen==pytest.approx(c.spring_length_mm,abs=1e-5)
