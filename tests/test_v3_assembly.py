import pytest
from modular_arm.v3.assembly import fasteners,spring,insertion_clearance
from modular_arm.v3.retained_core import parts
from modular_arm.v3.locking_core import Core,spring_space

def test_top_loading():
    assert insertion_clearance()==[]

@pytest.mark.parametrize("travel",[0.,3.,7.])
def test_spring_geometry(travel):
    p=parts(travel=travel)
    s=spring(travel=travel)
    assert s.isValid() and len(s.Solids())==1
    for name,part in p.items():
        assert s.intersect(part).Volume()<1e-6,name

def test_fasteners_envelope():
    p=parts()
    for name,s in fasteners().items():
        assert s.isValid() and len(s.Solids())==1
        for n in ("rotating","plunger","keeper"):
            assert s.intersect(p[n]).Volume()<1e-6,(name,n)
        assert s.intersect(p["stationary"]).Volume()>0
