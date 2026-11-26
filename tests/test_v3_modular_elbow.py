import pytest
import cadquery as cq
from itertools import combinations
from modular_arm.v3.modular_elbow import parts
from modular_arm.v3.assembly import fasteners

@pytest.mark.parametrize("angle,travel",[(a,0.) for a in range(-180,181,45)]+[(a,7.) for a in range(-180,181,15)])
def test_modular_elbow_poses(angle,travel):
    p=parts(angle,travel,connected=True)
    for n,s in p.items():
        assert s.isValid() and len(s.Solids())==1,n
    for (a,s),(b,t) in combinations(p.items(),2):
        assert s.intersect(t).Volume()<1e-6,(angle,travel,a,b)
    for a,s in fasteners().items():
        for n in ("rotating","moving_plug","moving_pin","fixed_plug","fixed_pin"):
            assert s.intersect(p[n]).Volume()<1e-6,(angle,a,n)

def test_integral_rotor_insertion():
    p=parts()
    for dz in range(0,41,2):
        q=p["rotating"].translate(cq.Vector(0,0,dz))
        for n in ("stationary","plunger"):
            assert q.intersect(p[n]).Volume()<1e-6,(dz,n)

def test_modular_lock():
    p=parts()
    assert p["rotating"].rotate((0,0,0),(0,0,1),10).intersect(p["plunger"]).Volume()>1e-3

@pytest.mark.parametrize("angle",[-179,-175,-170,-160,160,170,175,179])
def test_near_fold_clearance(angle):
    p=parts(angle,7.,connected=True)
    for (a,s),(b,t) in combinations(p.items(),2):
        assert s.intersect(t).Volume()<1e-6,(angle,a,b)

@pytest.mark.parametrize("angle",[0,45,90,180])
def test_keeper_lateral_insertion(angle):
    p=parts(angle)
    for name,sign in (("keeper_positive",1),("keeper_negative",-1)):
        for d in range(0,81,2):
            q=p[name].translate(cq.Vector(0,sign*d,0))
            for n in ("stationary","rotating","plunger"):
                assert q.intersect(p[n]).Volume()<1e-6,(angle,name,d,n)

def test_split_keeper_capture():
    p=parts(0,7.)
    lifted=p["rotating"].translate(cq.Vector(0,0,1))
    for name in ("keeper_positive","keeper_negative"):
        assert lifted.intersect(p[name]).Volume()>1e-3
