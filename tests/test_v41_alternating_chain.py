import numpy as np
import pytest
from modular_arm.v4 import alternating_chain as a

@pytest.mark.parametrize("angles",[(0.,0.,0.),(90.,90.,90.)])
def test_port_alignment_and_alternation(angles):
    indices=(6,)*4
    f=a.frames(angles,indices)
    z=a.m.flush_arm.AXES[1]
    port=np.array([-46.,0.,z,1.])
    for j in range(2):
        expected=f[j]@a.c.rotation(angles[j])@a.c.segment_transform(6)
        assert f[j+1]@port==pytest.approx(expected@port)
        assert f[j+1][2,2]==pytest.approx(-f[j][2,2])
        assert np.linalg.det(f[j+1][:3,:3])==pytest.approx(1.)

def test_two_levels():
    f=a.frames((0.,0.,0.),(6,)*4)
    # Port elevations, not joint origin elevations.
    z=a.m.flush_arm.AXES
    heights=[float((f[0]@np.array([0.,0.,z[1],1.]))[2])]
    heights.extend(float((t@np.array([0.,0.,z[5],1.]))[2]) for t in f)
    assert heights==pytest.approx([z[1],z[5],z[1],z[5]])
