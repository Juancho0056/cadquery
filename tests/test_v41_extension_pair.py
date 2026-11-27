import numpy as np
import pytest
from modular_arm.v4 import extension_pair as e

@pytest.mark.parametrize("side",["a","b"])
@pytest.mark.parametrize("clock",[0.,90.,180.,270.])
def test_datums(side,clock):
    depth=65.
    index=5 if side=="a" else 1
    cx,cy=e.ia.CENTERS[index]
    z=e.MOUTH_Z if side=="a" else 0.
    t=e.counterpart_transform(side,depth,clock)
    x=-151.+depth if side=="a" else 6.+e.MOUTH_Z-depth
    assert np.linalg.det(t[:3,:3])==pytest.approx(1.)
    assert t@np.array([cx,cy,z,1.])==pytest.approx([x,0,e.m.flush_arm.AXES[1 if side=="a" else 5],1.])
    assert t[:3,:3]@np.array([0.,0.,1.])==pytest.approx([1.,0.,0.])

def test_invalid_depth():
    with pytest.raises(ValueError):
        e.counterpart_transform("a",101.)
