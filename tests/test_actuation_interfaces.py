import pytest
from modular_arm.assembly.manual_actuation import ActuationConfig,key_geometry,actuator_geometry
from modular_arm.mechanisms.pivot_contact import make_contact_pawl
import cadquery as cq

@pytest.mark.parametrize("reverse",[False,True])
def test_key_pocket_has_constant_volume(reverse):
    c=ActuationConfig()
    p=c.enclosure.contact
    a=p.joint.cartridge
    z=a.head_thickness_mm+a.fork_wall_mm+a.axial_clearance_per_side_mm
    volumes=[]
    for angle in (0.,5.,10.,15.,20.):
        pawl=make_contact_pawl(p,release_deg=angle,reverse=reverse).translate(cq.Vector(0,0,z))
        pocket=key_geometry(c,reverse,angle,c.key_clearance_mm)
        volumes.append(pawl.cut(pocket).Volume())
    assert max(volumes)-min(volumes)<1e-5

@pytest.mark.parametrize("reverse",[False,True])
def test_key_clearance(reverse):
    c=ActuationConfig()
    k=key_geometry(c,reverse,0.)
    cavity=key_geometry(c,reverse,0.,c.key_clearance_mm)
    assert k.cut(cavity).Volume()<1e-7
