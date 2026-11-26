import os
import pytest

pytest.importorskip("trimesh")
pytest.importorskip("manifold3d")
from modular_arm.v4.integrated_arm import integrated_half,parts,audit

@pytest.fixture(scope="module")
def archive():
    a=os.environ.get("ARM_REFERENCE_ZIP")
    if not a:
        pytest.skip("set ARM_REFERENCE_ZIP")
    return a

@pytest.mark.parametrize("index",[1,5])
def test_integral_body(archive,index):
    m=integrated_half(archive,index)
    assert m.is_volume
    assert len(m.split(only_watertight=False))==1

@pytest.mark.parametrize("angle,travel",[(0,0),(45,0),(90,0),(30,7)])
def test_integral_pose(archive,angle,travel):
    p=parts(archive,angle,travel)
    assert audit(p)==[]
