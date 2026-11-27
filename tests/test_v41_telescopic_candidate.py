import numpy as np
import pytest
from modular_arm.v4.telescopic_candidate import shaft_transform,SHAFT_LENGTH,CENTERS
from modular_arm.v4.flush_arm import AXES

@pytest.mark.parametrize("angle",[0.,90.,180.,270.])
def test_tip_and_axis_mapping(angle):
    cx,cy=CENTERS[3]
    t=shaft_transform(60.,angle)
    assert np.linalg.det(t[:3,:3])==pytest.approx(1.)
    assert t@np.array([cx,cy,SHAFT_LENGTH,1.])==pytest.approx([60.,0.,AXES[5],1.])
    assert t@np.array([cx,cy,0.,1.])==pytest.approx([222.,0.,AXES[5],1.])
