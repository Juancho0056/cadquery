import numpy as np
import cadquery as cq
from modular_arm.v4.mesh_fit import cad_mesh,contacts

def test_surface_collision_positive_control():
    a=cad_mesh(cq.Solid.makeBox(10,10,10))
    b=cad_mesh(cq.Solid.makeBox(10,10,10,cq.Vector(5,5,5)))
    assert contacts(a,b)>0

def test_surface_collision_negative_control():
    a=cad_mesh(cq.Solid.makeBox(10,10,10))
    b=cad_mesh(cq.Solid.makeBox(10,10,10,cq.Vector(20,0,0)))
    assert contacts(a,b)==0

def test_containment_positive_control():
    from modular_arm.v4.mesh_fit import pair_report
    a=cad_mesh(cq.Solid.makeBox(10,10,10))
    b=cad_mesh(cq.Solid.makeBox(2,2,2,cq.Vector(4,4,4)))
    r=pair_report(a,b)
    assert r["surface_contacts"]==0
    assert r["b_vertices_inside_a"]>0

def test_separated_mate_negative_control():
    from modular_arm.v4.mesh_fit import pair_report,failures
    a=cad_mesh(cq.Solid.makeBox(10,10,10))
    b=cad_mesh(cq.Solid.makeBox(10,10,10,cq.Vector(20,0,0)))
    r={"original":"a","cad":"b","expected_mate":True,**pair_report(a,b)}
    assert r["sampled_gap_mm"]>1
    assert failures([r])[0]["reason"]=="mating_gap_above_screening_threshold"

def test_close_nonintersecting_mate():
    from modular_arm.v4.mesh_fit import pair_report,failures
    a=cad_mesh(cq.Solid.makeBox(10,10,10))
    b=cad_mesh(cq.Solid.makeBox(10,10,10,cq.Vector(10.3,0,0)))
    r={"original":"a","cad":"b","expected_mate":True,**pair_report(a,b)}
    assert not failures([r])
