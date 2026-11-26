import pytest
from modular_arm.v4.hardware_screen import spring_candidate,report

def test_spring_candidate():
    r=spring_candidate(18,23,5,.8)
    assert not r["issues"]
    assert r["preload_N"]==pytest.approx(8)
    assert r["full_press_N"]==pytest.approx(13.6)

def test_spring_rejects_incompatible_dimensions():
    r=spring_candidate(19,12,6,.8)
    assert len(r["issues"])==3

def test_hardware_nominal_geometry():
    r=report()
    for row in r["poses"]:
        assert all(v<1e-6 for k,v in row.items() if "intersection" in k),row
    for row in r["retainers"]["checks"]:
        assert row["intersection_a_mm3"]<1e-6
        assert row["intersection_b_mm3"]<1e-6
        assert row["axial_capture_test_mm3"]>0
