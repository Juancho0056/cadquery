import pytest
from modular_arm.assembly.manual_actuation import ActuationConfig,actuator_geometry,release_commands,manual_parts
from modular_arm.mechanisms.selector_contract import Mode

@pytest.mark.parametrize("mode",list(Mode))
def test_exclusive_command(mode):
    c=ActuationConfig()
    a,b=release_commands(c,mode)
    assert min(a,b)==0

@pytest.mark.parametrize("reverse",[False,True])
def test_actuator_is_solid(reverse):
    s=actuator_geometry(ActuationConfig(),reverse)
    assert s.isValid() and len(s.Solids())==1
