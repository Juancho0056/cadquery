"""Concentric core with independent radial journal and removable axial keeper.
Keeper fastening is represented by clearance holes, not installed hardware.
"""
import cadquery as cq
from modular_arm.v3.locking_core import Core,cyl,stationary,rotating,plunger,spring_space,parts as core_parts
import math

def ring(ro,ri,z,h):
    return cyl(ro,z,h).cut(cyl(ri,z-1,h+2))

def parts(c=None,angle=0.,travel=0.):
    c=c or Core()
    p=core_parts(c,angle,travel)
    r=c.housing_radius
    # Outer stationary sleeve and rotor shoulder form journal independent of teeth.
    p["stationary"]=p["stationary"].fuse(ring(r+4,r+.35,10,12).fuse(ring(r+4,r-1,10,2))).clean()
    # Cap overlaps the rotor radially by 3 mm, leaving axial running clearance.
    cap=ring(r+4,r-3,19,4)
    for a in range(0,360,90):
        t=math.radians(a+45)
        x,y=(r+2.1)*math.cos(t),(r+2.1)*math.sin(t)
        hole=cq.Solid.makeCylinder(1.6,5,cq.Vector(x,y,18.5))
        cap=cap.cut(hole)
        pilot=cq.Solid.makeCylinder(1.15,8,cq.Vector(x,y,14))
        p["stationary"]=p["stationary"].cut(pilot)
    # sleeve ends at z=22: cap must sit on top, not overlap sleeve.
    p["stationary"]=p["stationary"].cut(ring(r+5,r+.35,19,4)).clean()
    p["keeper"]=cap.clean()
    return p
