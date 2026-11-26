import pytest
from modular_arm.assembly.common_selector import CommonSelectorConfig,slider,slider_displacement,lever
from modular_arm.mechanisms.selector_contract import Mode

@pytest.mark.parametrize("mode",list(Mode))
def test_slider_solid(mode):
    c=CommonSelectorConfig()
    s=slider(c,slider_displacement(c,mode))
    assert s.isValid() and len(s.Solids())==1

@pytest.mark.parametrize("reverse",[False,True])
def test_lever_solid(reverse):
    c=CommonSelectorConfig()
    for q in (0.,10.,20.):
        s=lever(c,reverse,q)
        assert s.isValid() and len(s.Solids())==1
