from itertools import combinations
import cadquery as cq
import pytest
from modular_arm.assembly.serviceable_ratchet import ServiceableConfig,enclosure_parts,serviceable_parts

@pytest.fixture(scope="module")
def parts():
    return serviceable_parts()

def test_enclosure_valid():
    p=enclosure_parts()
    for s in p.values():
        assert s.isValid() and len(s.Solids())==1
    assert p["housing"].intersect(p["lid"]).Volume()<1e-6

def test_all_pairs(parts):
    for (a,p),(b,q) in combinations(parts.items(),2):
        assert p.intersect(q).Volume()<1e-6,(a,b)

@pytest.mark.parametrize("name",["lower_pivot","upper_pivot","lower_stop","upper_stop"])
def test_captured_support(parts,name):
    pin=parts[name]
    assert pin.translate(cq.Vector(0,0,1)).intersect(parts["lid"]).Volume()>1e-3
    assert pin.translate(cq.Vector(0,0,-1)).intersect(parts["housing"]).Volume()>1e-3

def test_lid_lifts_after_removing_axle_and_fasteners(parts):
    # Axle and angular lock must first be removed. Fasteners are removed too.
    for dz in (0,1,5,15,30):
        lid=parts["lid"].translate(cq.Vector(0,0,dz))
        for name,p in parts.items():
            if name in ("lid","axle","grooved_pin","snap_keeper","angular_lock") or name.startswith("lid_"):
                continue
            assert lid.intersect(p).Volume()<1e-6,(name,dz)
