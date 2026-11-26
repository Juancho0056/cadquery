"""Integrated low-profile A/B joint, independently modeled.
Rigid CAD prototype, not load rated. Radial screw envelopes are not threads.
"""
import math
import cadquery as cq
from modular_arm.v3.locking_core import Core,cyl,profile

DIAMETER=56.
ARM_END=51.
JOURNAL=23.
GROOVE_ROOT=21.5
GUIDE_TOP=17.
ROTOR_BOTTOM=12.4
TEETH_BOTTOM=17.4
TEETH_TOP=23.4
STROKE=7.

def ring(ro,ri,z,h):
    return cyl(ro,z,h).cut(cyl(ri,z-1,h+2))

def turn(s,angle):
    return s.rotate((0,0,0),(0,0,1),angle)

def neck(z,h):
    # Broad tapered transition, rounded outer end, integral with disc.
    p=cq.Workplane("XY").moveTo(15,-22).lineTo(43,-12).threePointArc((51,0),(43,12)).lineTo(15,22).close().extrude(h).translate((0,0,z)).val()
    return p

def fixed():
    c=Core()
    s=cyl(28,-10,22).fuse(turn(neck(-4,16),180))
    s=s.fuse(cyl(JOURNAL,11.8,5.2))
    s=s.cut(ring(24,GROOVE_ROOT,13.1,3.2))
    s=s.cut(cyl(19.4,-7,7.1)).cut(profile(c,0,18,c.clearance))
    # Positive lower button stop, spring passes through.
    s=s.fuse(ring(19.8,10,-3,2))
    return s.clean()

def radial_angles():
    return (90,180,270)

def radial_cylinder(radius,start,length,a):
    t=math.radians(a)
    d=cq.Vector(math.cos(t),math.sin(t),0)
    return cq.Solid.makeCylinder(radius,length,cq.Vector(start*d.x,start*d.y,14.7),d)

def moving():
    c=Core()
    s=cyl(28,12.4,14).fuse(neck(12.4,14))
    s=s.cut(cyl(JOURNAL+.35,12,5.4))
    s=s.cut(profile(c,17.4,6,c.clearance))
    s=s.cut(cyl(c.stem_radius+c.clearance,23.3,4))
    # The 0.4 mm gap above journal is intentional.
    for a in radial_angles():
        s=s.cut(radial_cylinder(1.25,21.8,12,a))
    return s.clean()

def button(travel=0.):
    if not math.isfinite(travel) or not 0<=travel<=STROKE:
        raise ValueError("invalid travel")
    c=Core()
    s=profile(c,6,17.4).fuse(cyl(c.stem_radius,23.3,11.1))
    return s.clean().translate(cq.Vector(0,0,-travel))

def spring_space(travel=0.):
    return cyl(9,-7,13-travel)

def screws(angle=0.):
    # Nominal smooth M3 dog-point core, recessed into outer wall.
    return {f"retainer_{i}":turn(radial_cylinder(1.2,22.,5.8,a),angle)
            for i,a in enumerate(radial_angles(),1)}

def parts(angle=0.,travel=0.,hardware=True):
    if not math.isfinite(angle) or abs(angle)>180:
        raise ValueError("invalid angle")
    if travel<6.4 and abs(angle/45-round(angle/45))>1e-8:
        raise ValueError("nonindexed pose requires released clutch")
    p={"arm_a":fixed(),"arm_b":turn(moving(),angle),"button":button(travel)}
    if hardware:
        p.update(screws(angle))
    return p
