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
