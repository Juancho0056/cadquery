import pytest
from modular_arm.v4.modular_structural_screen import report

@pytest.fixture(scope="module")
def result():
    return report()

@pytest.mark.parametrize("label",["a","b"])
def test_positive_sections(result,label):
    h=result["halves"][label]
    assert h["nominal_center_wall_mm"]==pytest.approx(2.55)
    assert h["axial_ligament_to_tenon_end_mm"]==pytest.approx(5.85)
    assert h["axial_ligament_to_root_mm"]==pytest.approx(5.85)
    for row in h["sections"]:
        for key in ("head","tenon"):
            assert min(row[key][k] for k in ("area_mm2","wy_mm3","wz_mm3"))>0
        assert 0<row["head_area_fraction"]<=1.001
        assert 0<row["tenon_area_fraction"]<=1.001

def test_cross_bore_removes_material(result):
    for h in result["halves"].values():
        row=next(r for r in h["sections"] if r["station_mm"]==38)
        assert row["tenon_area_fraction"]<1
        assert row["tenon_wy_fraction"]<1
        assert row["tenon_wz_fraction"]<1
