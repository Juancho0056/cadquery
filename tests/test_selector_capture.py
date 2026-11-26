import pytest
import cadquery as cq
from modular_arm.assembly.common_selector import CommonSelectorConfig,common_selector_parts,slider,slider_displacement,guide_parts
from modular_arm.mechanisms.selector_contract import Mode

@pytest.fixture(scope="module")
def neutral():
    return common_selector_parts(mode=Mode.B)

def test_upper_capture(neutral):
    p=neutral["common_selector"].translate(cq.Vector(0,0,1.))
    assert p.intersect(neutral["lid"]).Volume()>1e-3

def test_lower_capture(neutral):
    p=neutral["common_selector"].translate(cq.Vector(0,0,-1.))
    assert p.intersect(neutral["lid"]).Volume()>1e-3

@pytest.mark.parametrize("sign",[-1,1])
def test_travel_stop(sign):
    c=CommonSelectorConfig()
    bottom,_=guide_parts(c)
    s=slider(c,sign*c.stroke_mm).translate(cq.Vector(sign*1.,0,0))
    assert s.intersect(bottom).Volume()>1e-3

def test_angular_lock_clip_capture(neutral):
    p=neutral["angular_lock_grooved_pin"]
    q=neutral["angular_lock_snap_keeper"]
    assert p.intersect(q).Volume()<1e-6
    for dz in (-1,1):
        assert p.translate(cq.Vector(0,0,dz)).intersect(q).Volume()>1e-3
