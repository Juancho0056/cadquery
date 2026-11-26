import os
import numpy as np
import pytest
from modular_arm.v4.reference_preview import aligned_triangles,terminal_report,write_stl

def test_binary_stl_writer(tmp_path):
    p=tmp_path/"test.stl"
    write_stl(p,np.array([[[0,0,0],[1,0,0],[0,1,0]]]))
    assert p.stat().st_size==134

@pytest.fixture
def archive():
    a=os.environ.get("ARM_REFERENCE_ZIP")
    if not a:
        pytest.skip("set ARM_REFERENCE_ZIP for original mesh tests")
    return a

def test_original_alignment(archive):
    a=aligned_triangles(archive,1)
    b=aligned_triangles(archive,5)
    assert a[:,:,0].max()==pytest.approx(-55,abs=.05)
    assert b[:,:,0].min()==pytest.approx(55,abs=.05)
    assert np.linalg.norm(b[0,0]-b[0,1])==pytest.approx(np.linalg.norm(aligned_triangles(archive,5,90)[0,0]-aligned_triangles(archive,5,90)[0,1]))

def test_original_bore_variation(archive):
    r=terminal_report(archive)
    assert all(not row["center_in_material"] for row in r["1"])
    assert all(row["centered_clear_diameter"]>9.8 for row in r["1"])
    assert any(row["centered_clear_diameter"]<9.8 for row in r["5"])
