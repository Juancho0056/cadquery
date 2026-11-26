"""Integral V4 ends for original arm terminal geometry, experimental.
Local coordinates: reference longitudinal axis Z; transverse pin axis X.
No original meshes redistributed. Nominal pin gauges, not retained hardware.
"""
import cadquery as cq
from modular_arm.v4 import joint

PIN_DIAMETER=9.8
PIN_BORE=10.4
AXIS_DISTANCE=65.
A_CENTER_Z=4.
B_CENTER_Z=19.4

def box(x,y,z,dx,dy,dz):
    return cq.Solid.makeBox(dx,dy,dz,cq.Vector(x,y,z))

def pin_bore():
    return cq.Solid.makeCylinder(PIN_BORE/2,40,cq.Vector(-20,0,0),cq.Vector(1,0,0))

def clevis():
    # Obj1 tongue 11.5 thick; 12.1 gap. Bridge beyond rounded tip Z+10.
    a=box(-12.05,-14,-8,6,28,23)
    b=box(6.05,-14,-8,6,28,23)
    bridge=box(-12.05,-14,10.6,24.1,28,7.4)
    return a.fuse(b).fuse(bridge).cut(pin_bore()).clean()

def tongue():
    # Obj5 fork nominal gap 11.2; tongue 10.6, shoulder before fork tip.
    tip=cq.Solid.makeCylinder(8,10.6,cq.Vector(-5.3,0,0),cq.Vector(1,0,0))
    stem=box(-5.3,-8,-18,10.6,16,18)
    return tip.fuse(stem).cut(pin_bore()).clean()

def placed(s,moving,angle=0.):
    s=s.rotate((0,0,0),(0,1,0),90)
    s=s.translate(cq.Vector(AXIS_DISTANCE if moving else -AXIS_DISTANCE,0,B_CENTER_Z if moving else A_CENTER_Z))
    return joint.turn(s,angle) if moving else s

def parts(angle=0.,travel=0.,hardware=True):
    p=joint.parts(0.,travel,hardware=False)
    # Validate angular contract without silently accepting non-indexed lock.
    joint.parts(angle,travel,hardware=False)
    p["arm_a"]=p["arm_a"].fuse(placed(clevis(),False)).clean()
    p["arm_b"]=joint.turn(p["arm_b"].fuse(placed(tongue(),True)).clean(),angle)
    if hardware:
        p.update(joint.screws(angle))
    return p

def pin_gauge(moving,angle=0.):
    width=27.2 if moving else 24.1
    return placed(cq.Solid.makeCylinder(PIN_DIAMETER/2,width,cq.Vector(-width/2,0,0),cq.Vector(1,0,0)),moving,angle)

def interference_report(angle,travel=7.):
    from itertools import combinations
    p=parts(angle,travel)
    p["pin_a"]=pin_gauge(False)
    p["pin_b"]=pin_gauge(True,angle)
    return [(a,b,v) for (a,s),(b,t) in combinations(p.items(),2)
            if (v:=s.intersect(t).Volume())>1e-6]
