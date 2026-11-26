import pytest
import cadquery as cq
from modular_arm.assembly.common_selector import CommonSelectorConfig,slider,slider_displacement,index_pin
from modular_arm.mechanisms.selector_contract import Mode

@pytest.mark.parametrize("mode",list(Mode))
def test_index_pin_fits_and_obstructs_translation(mode):
    c=CommonSelectorConfig()
    pin=index_pin(c)
    s=slider(c,slider_displacement(c,mode))
    assert pin.isValid() and len(pin.Solids())==1
    assert pin.intersect(s).Volume()<1e-6
    for sign in (-1,1):
        assert pin.intersect(s.translate(cq.Vector(sign,0,0))).Volume()>1e-3

def test_index_pin_clears_when_withdrawn():
    c=CommonSelectorConfig()
    pin=index_pin(c).translate(cq.Vector(0,0,c.slider_height_mm+1.))
    for i in range(21):
        s=slider(c,-c.stroke_mm+2*c.stroke_mm*i/20)
        assert pin.intersect(s).Volume()<1e-6
