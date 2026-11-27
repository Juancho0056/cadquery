import pytest
from modular_arm.v4 import separation_variant as s

@pytest.mark.parametrize("delta",[0.,1.,2.,3.,4.])
def test_axis_spacing(delta):
    assert s.f.AXES[5]+s.shift(5,delta)-s.f.AXES[1]-s.shift(1,delta)==pytest.approx(28.4+delta)

@pytest.mark.parametrize("delta",[1.,4.])
def test_valid_cad(delta):
    for i in (1,5):
        h=s.head(i,delta)
        assert h.isValid() and len(h.Solids())==1
    b=s.button(delta,0.)
    assert b.isValid() and len(b.Solids())==1
    assert b.BoundingBox().zmax==pytest.approx(s.f.button(0.).BoundingBox().zmax+delta,abs=.001)

def test_invalid_delta():
    with pytest.raises(ValueError): s.shift(1,-1.)
