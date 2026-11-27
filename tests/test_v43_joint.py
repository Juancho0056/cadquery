import pytest
from modular_arm.v43 import joint as j

@pytest.fixture(scope="module")
def source():
    return j.templates()

def test_solid_parts(source):
    assert all(x.isValid() and len(x.Solids())==1 for x in source.values())

def test_poses(source):
    for angle,travel in ((0.,0.),(45.,0.),(90.,0.),(180.,0.),(15.,7.),(165.,7.)):
        assert not j.audit(j.parts(angle,travel,source=source))

def test_assembly_path(source):
    assert all(x["overlap_mm3"]<=j.TOL for x in j.insertion_samples(source))

def test_retention(source):
    assert any(x["lip_keeper_overlap_mm3"]>j.TOL and x["keeper_journal_overlap_mm3"]>j.TOL for x in j.retention_samples(source))

def test_magnet_coupon():
    assert j.magnet_coupon().isValid()
