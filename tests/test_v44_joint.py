import pytest
from modular_arm.v44 import joint as j
@pytest.fixture(scope="module")
def parts(): return j.templates()
def test_valid(parts):
    assert all(s.isValid() and len(s.Solids())==1 for s in parts.values())
def test_poses(parts):
    for a,t in ((0.,0.),(90.,0.),(180.,0.),(15.,7.)):
        assert not j.audit(j.parts(a,t,parts))
def test_path(parts):
    assert all(x["overlap_mm3"]<=j.TOL for x in j.path_samples(parts))
def test_retention(parts):
    assert min(j.retention(parts).values())>j.TOL
