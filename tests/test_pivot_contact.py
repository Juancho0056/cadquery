import pytest
from itertools import combinations
from modular_arm.mechanisms.pivot_contact import PivotContactConfig,make_contact_rotor,make_contact_pawl,make_contact_stop
from modular_arm.assembly.ratchet import ratchet_parts

SAMPLES=[(0,0),(3,.75),(6,4),(9,7.5),(12,12),(14,14.5),(15,0)]

@pytest.fixture(scope="module")
def cfg():
    return PivotContactConfig()

@pytest.fixture(scope="module")
def rotor(cfg):
    return make_contact_rotor(cfg,lock_holes=False)

@pytest.mark.parametrize("reverse",[False,True])
def test_prescribed_contact_path(cfg,rotor,reverse):
    for a,q in SAMPLES:
        p=make_contact_pawl(cfg,release_deg=q,reverse=reverse)
        r=rotor.rotate((0,0,0),(0,0,1),-a if reverse else a)
        assert p.intersect(r).Volume()<1e-6,(a,q)
        assert p.intersect(make_contact_stop(cfg,reverse)).Volume()<1e-6,(a,q)

@pytest.mark.parametrize("reverse",[False,True])
def test_fixed_pawl_blocks_reverse_and_stop_blocks_closing(cfg,rotor,reverse):
    p=make_contact_pawl(cfg,reverse=reverse)
    r=rotor.rotate((0,0,0),(0,0,1),1 if reverse else -1)
    assert p.intersect(r).Volume()>1e-3
    x=-cfg.pivot_x_mm if reverse else cfg.pivot_x_mm
    closing=p.rotate((x,cfg.pivot_y_mm,0),(x,cfg.pivot_y_mm,1),-1 if reverse else 1)
    assert closing.intersect(make_contact_stop(cfg,reverse)).Volume()>1e-3

@pytest.mark.parametrize("angle,lower,upper,lock",[(0,0,0,True),(9,7.5,20,False),(-9,20,7.5,False)])
def test_integrated_prescribed_pose(cfg,angle,lower,upper,lock):
    parts=ratchet_parts(cfg,angle_deg=angle,lower_release=lower,upper_release=upper,lock_inserted=lock)
    for name,p in parts.items():
        assert p.isValid() and len(p.Solids())==1,name
    for (a,p),(b,q) in combinations(parts.items(),2):
        assert p.intersect(q).Volume()<1e-6,(a,b)
