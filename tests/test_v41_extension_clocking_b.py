import numpy as np
import pytest
from modular_arm.v4 import extension_clocking_b as b
@pytest.mark.parametrize("clock,sign",[(90.,1),(270.,-1)])
def test_axis(clock,sign):
    z=b.m.flush_arm.AXES[5]
    t=b.transform_b(clock)
    assert t@np.array([100.,0.,z,1.])==pytest.approx([100.,0.,z,1.])
    assert t@np.array([100.,0.,z+5.,1.])==pytest.approx([100.,-5.*sign,z,1.])
    assert np.linalg.det(t[:3,:3])==pytest.approx(1.)
