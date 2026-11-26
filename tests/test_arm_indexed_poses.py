import pytest
from itertools import combinations
from modular_arm.assembly.common_selector import common_selector_parts
from modular_arm.mechanisms.selector_contract import Mode

@pytest.mark.parametrize("angle",[-30.,-15.,0.,15.,30.])
def test_complete_indexed_pose(angle):
    parts=common_selector_parts(mode=Mode.B,include_arms=True,rotor_angle=angle)
    for name,p in parts.items():
        assert p.isValid() and len(p.Solids())==1,name
    for (a,p),(b,q) in combinations(parts.items(),2):
        assert p.intersect(q).Volume()<1e-6,(angle,a,b)

def test_reject_nonindexed_lock():
    with pytest.raises(ValueError):
        common_selector_parts(rotor_angle=7.)

def test_removed_angular_lock():
    parts=common_selector_parts(include_arms=True,lock_inserted=False)
    assert not any(name.startswith("angular_lock") for name in parts)
