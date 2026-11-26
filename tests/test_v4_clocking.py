import numpy as np
import pytest
from modular_arm.v4.clocking import matrix
@pytest.mark.parametrize("angle",[0.,90.,180.,270.])
def test_rigid_common_transform(angle):
    r=matrix(angle)[:3,:3]
    assert np.array_equal(r.T@r,np.eye(3))
    assert np.linalg.det(r)==pytest.approx(1.)
    assert np.array_equal(r@np.array([1,0,0]),[1,0,0])
    # A registered pin and hole retain relative position under common rotation.
    pin=np.array([10.,2.,-8.])
    hole=np.array([10.,2.,-7.8])
    assert np.linalg.norm(r@(pin-hole))==pytest.approx(.2)

def test_flip_outward():
    assert np.array_equal(matrix(180.)@np.array([0,0,-20,1]),[0,0,20,1])

@pytest.mark.parametrize("angle",[0,45,90,135,180])
def test_clocked_integrated_pose(angle):
    import os
    from modular_arm.v4.integrated_arm import parts,audit
    archive=os.environ.get("ARM_REFERENCE_ZIP")
    if not archive:
        pytest.skip("set ARM_REFERENCE_ZIP")
    assert audit(parts(archive,angle,0.,flush=True,clocking=180.))==[]
