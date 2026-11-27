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
    for label,h in result["halves"].items():
        row=next(r for r in h["sections"] if r["station_mm"]==38)
        assert row["tenon_area_fraction"]<1
        # Removing material reduces centroidal second moments, but W=I/c
        # need not decrease when the centroid and extreme-fiber distance move.
        cut,ref=row["tenon"],row["unbored_tenon"]
        for key in ("iy_mm4","iz_mm4"):
            assert 0<cut[key]<ref[key]
        assert row["tenon_wz_fraction"]<1
        # Verify W_y independently against the extreme fibers of the D profile.
        # The transverse bore does not remove its top or bottom extremities.
        from modular_arm.v4 import modular_connection as m
        axis=m.flush_arm.AXES[1 if label=="a" else 5]
        for section in (cut,ref):
            c=max(abs(axis-m.RADIUS-section["centroid_z_mm"]),
                  abs(axis+m.FLAT-section["centroid_z_mm"]))
            assert section["wy_mm3"]==pytest.approx(section["iy_mm4"]/c,rel=.002)
