from itertools import combinations
import pytest
import cadquery as cq
from modular_arm.config.joint import JointConfig
from modular_arm.components.joint import make_fork, make_retained_axle
from modular_arm.assembly.joint import joint_parts
from modular_arm.interfaces.joint import RotorInterface

def test_fork_connected():
    j=JointConfig()
    s=make_fork(j).val()
    assert s.isValid() and len(s.Solids())==1
    assert s.BoundingBox().zlen==pytest.approx(j.stack_height_mm,abs=1e-5)

def test_cross_bore_open():
    j=JointConfig()
    c=j.cartridge
    probe=cq.Solid.makeCylinder(j.pin_diameter_mm/2,c.axle_diameter_mm+2,
        cq.Vector(-c.axle_diameter_mm/2-1,0,j.pin_center_z_mm),cq.Vector(1,0,0))
    assert make_retained_axle(j).val().intersect(probe).Volume()<1e-6

def test_static_interference():
    for (a,x),(b,y) in combinations(joint_parts().items(),2):
        assert x.intersect(y).Volume()<1e-6,(a,b)

def test_rotor_interface():
    j=JointConfig()
    r=RotorInterface.from_joint(j)
    assert r.tooth_count==24 and r.pitch_degrees==15
    assert r.axial_thickness_mm<j.cartridge.fork_gap_mm
    assert r.envelope_diameter_mm/2<j.bridge_inner_x_mm

def test_axial_stop_contacts_when_axle_is_pulled():
    j=JointConfig()
    p=joint_parts(j)
    # Pull toward the axle head; the cross pin must intersect the upper plate.
    shifted=p["cross_pin"].translate(cq.Vector(0,0,-j.cartridge.retention_extension_mm/2))
    assert shifted.intersect(p["fork"]).Volume()>1e-3
