import os
import pytest
from modular_arm.v4 import modular_connection as m
from modular_arm.v4.integrated_arm import audit

@pytest.mark.parametrize("i",[1,5])
def test_socket_fit_and_insertion(i):
    h=m.head(i)
    a=m.coupon_arm(i)
    assert h.intersect(a).Volume()<1e-6
    assert h.intersect(m.pin_gauge(i)).Volume()<1e-6
    assert a.intersect(m.pin_gauge(i)).Volume()<1e-6
    for shift in (1,4,8,12,16,20):
        moved=a.translate((m.direction(i)*shift,0,0))
        assert h.intersect(moved).Volume()<1e-6
    assert a.translate((m.direction(i)*2,0,0)).intersect(m.pin_gauge(i)).Volume()>0

@pytest.mark.parametrize("angle,travel",[(0,0),(90,0),(180,0),(30,7)])
def test_coupon_poses(angle,travel):
    assert audit(m.parts(None,angle,travel))==[]

@pytest.mark.parametrize("angle,travel",[(0,0),(180,0),(30,7)])
def test_reference_poses(angle,travel):
    archive=os.environ.get("ARM_REFERENCE_ZIP")
    if not archive:
        pytest.skip("set ARM_REFERENCE_ZIP")
    assert audit(m.parts(archive,angle,travel))==[]

@pytest.mark.parametrize("i",[1,5])
def test_m4_hardware_envelopes(i):
    from itertools import combinations
    h=m.head(i)
    a=m.coupon_arm(i)
    hardware=m.connection_hardware(i)
    for label,s in hardware.items():
        assert s.isValid() and len(s.Solids())==1,label
        assert h.intersect(s).Volume()<1e-6,label
        assert a.intersect(s).Volume()<1e-6,label
    for (na,sa),(nb,sb) in combinations(hardware.items(),2):
        assert sa.intersect(sb).Volume()<1e-6,(na,nb)
    assert m.PIN_RADIUS==pytest.approx(2.15)
    assert m.hardware_dimensions()["thread_projection_beyond_nut_mm"]>=1.4
    assert m.hardware_dimensions()["nominal_socket_side_wall_at_seat_center_mm"]>=2.5

@pytest.mark.parametrize("i",[1,5])
def test_m4_seats_do_not_change_core(i):
    original=m.flush_arm.finished_flush(i)
    core=m.axial_box(i,-60,28)
    assert original.intersect(core).cut(m.head(i)).Volume()<1e-6
