import numpy as np
import pytest
from modular_arm.v4 import extension_lock as l

@pytest.mark.parametrize("side",["a","b"])
def test_transverse_axis(side):
    t=l.screw_transform(side,65.,0.)
    cx,cy=l.SCREW_CENTER
    p=t@np.array([cx,cy,0.,1.])
    q=t@np.array([cx,cy,10.,1.])
    assert np.linalg.det(t[:3,:3])==pytest.approx(1.)
    assert p[1]==pytest.approx(0.)
    assert q-p==pytest.approx([0.,0.,-10.,0.])
    assert p[2]==pytest.approx(l.e.m.flush_arm.AXES[1 if side=="a" else 5]+l.BOSS_TOP_X-l.e.ia.CENTERS[5][0])
