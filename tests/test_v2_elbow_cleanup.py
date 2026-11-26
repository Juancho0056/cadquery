import pytest
from modular_arm.v2.elbow import Elbow,fixed_half,moving_half

@pytest.mark.parametrize("builder",[fixed_half,moving_half])
def test_clean_single_solid(builder):
    p=builder(Elbow())
    q=p.clean()
    assert p.isValid() and len(p.Solids())==1
    assert len(p.Faces())==len(q.Faces())
    assert p.Volume()==pytest.approx(q.Volume(),abs=1e-6)
