import pytest
from itertools import combinations
from modular_arm.v4.prototype_coupon import parts,printable_parts
from modular_arm.v4 import flush_arm

def test_coupon_preserves_joint():
    p=parts()
    for i,n in ((1,"arm_a_coupon"),(5,"arm_b_coupon")):
        original=flush_arm.finished_flush(i)
        s=p[n]
        assert s.isValid() and len(s.Solids())==1
        assert s.cut(original).Volume()<1e-6
        assert s.Volume()<original.Volume()
    for a,b in combinations(p.values(),2):
        assert a.intersect(b).Volume()<1e-6

def test_coupon_print_orientation():
    for s in printable_parts().values():
        assert s.BoundingBox().zmin==pytest.approx(0,abs=1e-6)
        assert s.isValid()
