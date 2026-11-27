import pytest
from modular_arm.v45 import joint as j
@pytest.fixture(scope="module")
def parts(): return j.templates()
def test_valid(parts):
    assert all(s.isValid() and len(s.Solids())==1 for s in parts.values())
def test_width(parts):
    for n in ("head_a","head_b","cap"):
        assert parts[n].BoundingBox().ylen<=45.001
def test_poses(parts):
    for a,t in ((0.,0.),(90.,0.),(180.,0.),(15.,7.)):
        assert not j.audit(j.parts(a,t,parts))
def test_path(parts):
    assert all(x["overlap_mm3"]<=j.TOL for x in j.path_samples(parts))
def test_retention(parts):
    r=j.retention(parts)
    assert r.pop("unlocked_pull_clear")<=j.TOL
    assert min(r.values())>j.TOL


def test_magnetic_variant():
    p=j.templates("magnet")
    assert all(s.isValid() and len(s.Solids())==1 for s in p.values())
    rows=j.magnetic_checks(p)
    assert all(not row["collisions"] and row["face_gap_mm"]>=2.19 for row in rows)
