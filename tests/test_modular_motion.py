"""Full assembly sampled motion; no automatic dynamics claimed."""
from itertools import combinations
import pytest
from modular_arm.assembly.common_selector import common_selector_parts
from modular_arm.mechanisms.selector_contract import Mode

# Previously validated contact-path samples for the isolated pivot mechanism.
SAMPLES=((0.,0.),(3.,.75),(6.,4.),(9.,7.5),(12.,12.),(14.,14.5),(15.,0.))

@pytest.mark.parametrize("mode,sign",[(Mode.A,-1),(Mode.C,1)])
def test_complete_modular_one_pitch(mode,sign):
    for angle,release in SAMPLES:
        p=common_selector_parts(mode=mode,include_arms=True,include_stops=True,
            include_extensions=True,lock_inserted=False,rotor_angle=sign*angle,
            active_release=release)
        for name,s in p.items():
            assert s.isValid() and len(s.Solids())==1,(angle,name)
        for (a,piece),(b,other) in combinations(p.items(),2):
            assert piece.intersect(other).Volume()<1e-6,(mode,angle,release,a,b)

def test_invalid_release_with_lock():
    with pytest.raises(ValueError):
        common_selector_parts(mode=Mode.A,active_release=5.)

def test_invalid_release_in_neutral():
    with pytest.raises(ValueError):
        common_selector_parts(mode=Mode.B,active_release=5.,lock_inserted=False)
