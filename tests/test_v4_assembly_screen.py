import pytest
import cadquery as cq
from modular_arm.v4.assembly_screen import overlap,report

def test_overlap_positive_and_negative():
    a=cq.Solid.makeBox(10,10,10)
    assert overlap(a,a.translate((5,0,0)))==pytest.approx(500)
    assert overlap(a,a.translate((11,0,0)))<1e-6

@pytest.fixture(scope="module")
def result():
    return report()

def test_button_positive_capture(result):
    assert any(r["rotor_intersection_mm3"]>1e-6 for r in result["button_capture"])

def test_nominal_assembly_path(result):
    assert result["issues"]==[]
