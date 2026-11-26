import pytest
from itertools import combinations
from modular_arm.v4.reference_ends import parts,clevis,tongue,pin_gauge,interference_report

@pytest.mark.parametrize("angle,travel",[(0,0),(45,0),(-45,0),(90,0),(-90,0),(0,7),(30,7),(-30,7),(90,7),(-90,7)])
def test_integral_reference_ends(angle,travel):
    p=parts(angle,travel)
    for n,s in p.items():
        assert s.isValid() and len(s.Solids())==1,n
    assert interference_report(angle,travel)==[]

def test_nominal_end_gauges():
    for moving in (False,True):
        s=parts(0,0)["arm_b" if moving else "arm_a"]
        assert s.intersect(pin_gauge(moving)).Volume()<1e-6

def test_interface_dimensions():
    assert clevis().BoundingBox().xlen==pytest.approx(24.1)
    assert tongue().BoundingBox().xlen==pytest.approx(10.6)
