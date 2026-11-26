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
