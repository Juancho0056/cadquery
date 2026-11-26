"""V3 retained core with integral offset modular sockets.
Offset bridge clears stationary keeper and screw heads; experimental, not load rated.
"""
import math
import cadquery as cq
from modular_arm.v2.connector import Port,box,socket,plug,locking_pin
from modular_arm.v3.locking_core import Core
from modular_arm.v3.retained_core import parts as retained_parts

MOUTH=70.
FIXED_Z=2.2
MOVING_Z=29.5

def pose(shape,angle):
    return shape.rotate((0,0,0),(0,0,1),angle)

def placed(shape,moving,angle=0.):
    if moving:
        return pose(shape.translate(cq.Vector(MOUTH,0,MOVING_Z)),angle)
    return pose(shape,180).translate(cq.Vector(-MOUTH,0,FIXED_Z))

def parts(angle=0.,travel=0.,*,connected=False):
    c=Core()
    port=Port()
    p=retained_parts(c,0.,travel)
    if not math.isfinite(angle) or abs(angle)>180:
        raise ValueError("angle outside prototype range")
    if travel<6.4 and abs(angle/45-round(angle/45))>1e-8:
        raise ValueError("nonindexed pose requires released clutch")
    # Stationary neck joins cup below the rotating journal.
    fixed_neck=box(-43,15,24,18).translate(cq.Vector(0,0,0))
    p["stationary"]=p["stationary"].fuse(fixed_neck).fuse(placed(socket(port),False)).clean()
    # Integral riser inside keeper opening; bridge passes above screw heads.
    riser=box(19,6,10,10).translate(cq.Vector(0,0,25.5))
    bridge=box(19,24,12,8).translate(cq.Vector(0,0,30.5))
    moving=p["rotating"].fuse(riser).fuse(bridge).fuse(placed(socket(port),True)).clean()
    p["rotating"]=pose(moving,angle)
    for name in ("stationary","rotating"):
        if not p[name].isValid() or len(p[name].Solids())!=1:
            raise ValueError("invalid integral housing: "+name)
    if connected:
        for side,moving_side in (("fixed",False),("moving",True)):
            p[side+"_plug"]=placed(plug(port),moving_side,angle)
            p[side+"_pin"]=placed(locking_pin(port),moving_side,angle)
    return p
