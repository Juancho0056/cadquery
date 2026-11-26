import pytest
import cadquery as cq
from itertools import combinations
from modular_arm.v3.modular_elbow import parts,keeper_insertion_audit,assembly_parts,ASSEMBLY_ANGLE
from modular_arm.v3.assembly import fasteners

@pytest.mark.parametrize("angle,travel",[(a,0.) for a in range(-180,181,45)]+[(a,7.) for a in range(-180,181,15)])
def test_modular_elbow_poses(angle,travel):
    p=parts(angle,travel,connected=True)
    for n,s in p.items():
        assert s.isValid() and len(s.Solids())==1,n
    for (a,s),(b,t) in combinations(p.items(),2):
        assert s.intersect(t).Volume()<1e-6,(angle,travel,a,b)
    for a,s in fasteners().items():
        for n in ("rotating","moving_plug","moving_pin","fixed_plug","fixed_pin","fixed_retainer","moving_retainer"):
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

@pytest.mark.parametrize("angle",[0,180])
def test_keeper_lateral_insertion(angle):
    assert keeper_insertion_audit(angle,step=1.)==[]

@pytest.mark.parametrize("angle",[45,90])
def test_keeper_wrong_orientation_detected(angle):
    # These are operating poses, not supported insertion poses.
    # Preserve the observed interference as a negative control.
    defects=keeper_insertion_audit(angle,step=2.)
    assert any(a=="keeper_positive" and b=="rotating" for a,b,d,v in defects)

def test_assembly_contract():
    assert ASSEMBLY_ANGLE==0
    p=assembly_parts()
    assert keeper_insertion_audit()==[]
    assert p["rotating"].intersect(p["plunger"]).Volume()<1e-6

def test_split_keeper_capture():
    p=parts(0,7.)
    lifted=p["rotating"].translate(cq.Vector(0,0,1))
    for name in ("keeper_positive","keeper_negative"):
        assert lifted.intersect(p[name]).Volume()>1e-3

@pytest.mark.parametrize("angle",range(0,181,10))
def test_post_assembly_motion(angle):
    p=parts(angle,7.,connected=True)
    for (a,s),(b,t) in combinations(p.items(),2):
        assert s.intersect(t).Volume()<1e-6,(angle,a,b)

def test_keeper_wrong_angle_negative_control_135():
    assert keeper_insertion_audit(135,step=2.)
