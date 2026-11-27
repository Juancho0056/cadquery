import numpy as np
import pytest
from modular_arm.v4 import extension_clocking as c

def test_axis_and_flip():
    z=c.m.flush_arm.AXES[1]
    t=c.module_transform(180.)
    assert t@np.array([-46.,0.,z,1.])==pytest.approx([-46.,0.,z,1.])
    assert t@np.array([-100.,3.,z+7.,1.])==pytest.approx([-100.,-3.,z-7.,1.])
    assert np.linalg.det(t[:3,:3])==pytest.approx(1.)
    assert t@t==pytest.approx(np.eye(4))

def test_baseline():
    assert c.module_transform(0.)==pytest.approx(np.eye(4))
