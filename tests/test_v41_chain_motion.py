import pytest
from modular_arm.v4 import alternating_chain as a

def test_nonindexed_frame():
    f=a.frames((15.,0.),(6,6,6))
    assert len(f)==2
    assert f[1][2,2]==pytest.approx(-1.)

def test_invalid_angle():
    with pytest.raises(ValueError): a.frames((181.,0.),(6,6,6))
