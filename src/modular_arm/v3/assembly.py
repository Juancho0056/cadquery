"""Assembly diagnostics for retained core; nominal hardware envelopes, not thread geometry."""
import math
import cadquery as cq
from modular_arm.v3.locking_core import Core,cyl,spring_space
from modular_arm.v3.retained_core import parts as retained_parts

def fasteners(c=None):
    c=c or Core()
    result={}
    for i in range(4):
        a=math.radians(45+90*i)
        x,y=(c.housing_radius+3.7)*math.cos(a),(c.housing_radius+3.7)*math.sin(a)
        # M3 x 8 under head, nominal cylindrical thread envelope.
        shaft=cyl(1.5,15,8)
        head=cyl(2.75,23,3)
        result[f"m3_{i+1}"]=shaft.fuse(head).translate(cq.Vector(x,y,0)).clean()
    return result

def spring(c=None,travel=0.,wire=.8,turns=6):
    """Illustrative helical wire, not a selected or validated commercial spring."""
    c=c or Core()
    if not 0<=travel<=c.stroke or wire<=0 or turns<3:
        raise ValueError("invalid spring pose")
    length=13-travel
    if length<=wire*(turns+1):
        raise ValueError("coil bind")
    radius=(c.spring_od-wire)/2
    h=length-wire
    path=cq.Wire.makeHelix(h/turns,h,radius)
    # Tangent to helix at start: y and z components.
    tangent=cq.Vector(0,2*math.pi*radius,h/turns)
    plane=cq.Plane(origin=(radius,0,0),normal=tangent)
    return cq.Workplane(plane).circle(wire/2).sweep(path,isFrenet=True).val().translate(cq.Vector(0,0,-7+wire/2))

def insertion_poses(c=None):
    """Top-loading assembly: cup, spring, plunger, rotor, keeper, screws.
    Rotor must be indexed during insertion. Screw pilots intentionally intersect
    the nominal threads; self-tapping suitability requires physical qualification.
    """
    c=c or Core()
    p=retained_parts(c)
    yield "plunger",p["plunger"],{"stationary":p["stationary"]}
    yield "rotating",p["rotating"],{"stationary":p["stationary"],"plunger":p["plunger"]}
    yield "keeper",p["keeper"],{"stationary":p["stationary"],"plunger":p["plunger"],"rotating":p["rotating"]}

def insertion_clearance(c=None,steps=range(0,41,2)):
    result=[]
    for name,part,obstacles in insertion_poses(c):
        for dz in steps:
            q=part.translate(cq.Vector(0,0,dz))
            for other,s in obstacles.items():
                v=q.intersect(s).Volume()
                if v>1e-6:
                    result.append((name,other,dz,v))
    return result
