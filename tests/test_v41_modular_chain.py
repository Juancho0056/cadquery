import numpy as np
import pytest
from modular_arm.v4 import modular_chain as c

@pytest.mark.parametrize("index",[0,6,12])
def test_segment_registration(index):
    # Source inner tip Z=0 maps from canonical inner to counterpart B.
    t=c.segment_transform(index)
    cx,cy=c.ia.CENTERS[1]
    p=np.array([-c.ia.JOIN_X-c.ia.CUTS[1],0.,c.m.flush_arm.AXES[1],1.])
    q=c.e.counterpart_transform("b",c.l.indexed_depth(index),0.)@np.array([cx,cy,0.,1.])
    assert t@p==pytest.approx(q)

def test_chain_frames():
    f=c.frames((90.,-90.,0.),(6,6,6,6))
    assert len(f)==3
    assert f[1]==pytest.approx(c.rotation(90.)@c.segment_transform(6))
    assert f[2]==pytest.approx(f[1]@c.rotation(-90.)@c.segment_transform(6))

def test_invalid_index():
    with pytest.raises(ValueError): c.segment_transform(13)
    with pytest.raises(ValueError): c.frames((0.,0.),(6,6))
