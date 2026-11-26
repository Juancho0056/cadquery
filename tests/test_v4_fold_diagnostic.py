import pytest
pytest.importorskip("manifold3d")
from modular_arm.v4.integrated_arm import mesh_library
from modular_arm.v4.fold_diagnostic import report_collision

def test_collision_location():
    tm=mesh_library()
    a=tm.creation.box(extents=(10,10,10))
    a.apply_translation((-60,0,0))
    b=a.copy()
    b.apply_translation((5,0,0))
    r,q=report_collision(a,b)
    assert r["volume_mm3"]==pytest.approx(500)
    assert r["region"]=="longitudinal"
    assert r["bounds_mm"][0]==pytest.approx([-60,-5,-5])

def test_no_collision():
    tm=mesh_library()
    a=tm.creation.box()
    b=a.copy()
    b.apply_translation((3,0,0))
    r,q=report_collision(a,b)
    assert r is None
