import pytest
import cadquery as cq
from modular_arm.v4.arm_interface import bearing_coupon,journal_gauge
from modular_arm.v4.reference_arm import read_part,journal_report
import os

@pytest.mark.parametrize("clearance",[.15,.25,.5])
def test_coupon(clearance):
    s=bearing_coupon(clearance)
    g=journal_gauge()
    assert s.isValid() and len(s.Solids())==1
    for angle in (0,45,90,180):
        q=g.rotate((0,0,0),(0,0,1),angle)
        assert q.intersect(s).Volume()<1e-6
    assert g.translate(cq.Vector(clearance+.2,0,0)).intersect(s).Volume()>1e-3

def test_reference_measurements():
    archive=os.environ.get("ARM_REFERENCE_ZIP")
    if not archive:
        pytest.skip("original user archive not redistributed; set ARM_REFERENCE_ZIP")
    p=read_part(archive,3)
    lo,hi=p.bounds
    assert hi[2]-lo[2]==pytest.approx(162,abs=.1)
    for row in journal_report(archive):
        assert row["diameter_min"]==pytest.approx(12,abs=.02)
        assert row["diameter_max"]==pytest.approx(12,abs=.02)
