import pytest
import cadquery as cq
from itertools import combinations
from modular_arm.v3.modular_elbow import parts,keeper_insertion_audit
from modular_arm.v3.assembly import fasteners
from modular_arm.v2.connector import Port

@pytest.mark.parametrize("angle,travel",[(a,0.) for a in range(-180,181,45)]+[(a,7.) for a in range(-180,181,15)])
def test_shorter_ports(angle,travel):
    p=parts(angle,travel,connected=True,mouth=67.)
    for n,s in p.items():
        assert s.isValid() and len(s.Solids())==1,n
    for (a,s),(b,t) in combinations(p.items(),2):
        assert s.intersect(t).Volume()<1e-6,(angle,travel,a,b)
    for n,s in fasteners().items():
        for name in ("rotating","moving_plug","moving_pin","moving_retainer","fixed_plug","fixed_pin","fixed_retainer"):
            assert s.intersect(p[name]).Volume()<1e-6,(angle,n,name)

def test_shorter_ports_assembly():
    assert keeper_insertion_audit(mouth=67.)==[]

def test_shorter_ports_size():
    base=parts(0,0)
    short=parts(0,0,mouth=67.)
    assert short["rotating"].BoundingBox().xmax==pytest.approx(67.)
    assert short["stationary"].BoundingBox().xmin==pytest.approx(-67.)
    for name in ("plunger","keeper_positive","keeper_negative"):
        assert base[name].Volume()==pytest.approx(short[name].Volume(),abs=1e-6)

def test_shorter_ports_positive_lock():
    p=parts(mouth=67.)
    assert p["rotating"].rotate((0,0,0),(0,0,1),10).intersect(p["plunger"]).Volume()>1e-3

def test_unsupported_compaction():
    with pytest.raises(ValueError):
        parts(mouth=60.)
