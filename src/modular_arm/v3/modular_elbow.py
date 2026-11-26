"""V3 retained core with integral offset modular sockets.
Offset bridge clears stationary keeper and screw heads; experimental, not load rated.
"""
import math
import cadquery as cq
from modular_arm.v2.connector import Port,box,socket,plug
from modular_arm.v3.locking_core import Core
from modular_arm.v3.retained_core import parts as retained_parts

MOUTH=70.
FIXED_Z=2.2
MOVING_Z=29.5

def pose(shape,angle):
    return shape.rotate((0,0,0),(0,0,1),angle)

def placed(shape,moving,angle=0.,*,mouth=MOUTH):
    if moving:
        return pose(shape.translate(cq.Vector(mouth,0,MOVING_Z)),angle)
    return pose(shape,180).translate(cq.Vector(-mouth,0,FIXED_Z))

# Groove and flush M3 retaining screw are located inside the head-side
# socket wall, outside the nominal plug/wall shear interfaces.
GROOVE_WIDTH=3.2
GROOVE_RADIUS=1.75
RETAINER_TIP_Y=2.0
RETAINER_LENGTH=12.
RETAINER_CORE_RADIUS=1.2
RETAINER_PILOT_RADIUS=1.25

def head_side(shape,moving):
    return shape if moving else shape.rotate((0,0,0),(1,0,0),180)

def groove_z(c):
    return c.outer_height/2-2.

def retained_socket(c,moving):
    z=groove_z(c)
    pilot=cq.Solid.makeCylinder(RETAINER_PILOT_RADIUS,c.outer_width/2,
        cq.Vector(-c.pin_center,RETAINER_TIP_Y-.2,z),cq.Vector(0,1,0))
    return socket(c).cut(head_side(pilot,moving)).clean()

def retainer_screw(c,moving):
    """M3x12 nominal core envelope; thread engagement not represented."""
    return head_side(cq.Solid.makeCylinder(RETAINER_CORE_RADIUS,RETAINER_LENGTH,
        cq.Vector(-c.pin_center,RETAINER_TIP_Y,groove_z(c)),cq.Vector(0,1,0)),moving)

def compact_pin(c, moving):
    """Full socket-wall engagement, with head facing away from folding gap."""
    half=c.outer_height/2
    x=-c.pin_center
    shaft=cq.Solid.makeCylinder(c.pin/2,c.outer_height+.5,cq.Vector(x,0,-half-.5))
    head=cq.Solid.makeCylinder(c.pin,3.,cq.Vector(x,0,half))
    z=groove_z(c)-GROOVE_WIDTH/2
    outer=cq.Solid.makeCylinder(c.pin/2+.1,GROOVE_WIDTH,cq.Vector(x,0,z))
    inner=cq.Solid.makeCylinder(GROOVE_RADIUS,GROOVE_WIDTH,cq.Vector(x,0,z))
    pin=shaft.fuse(head).cut(outer.cut(inner)).clean()
    return head_side(pin,moving)

def split_keeper(keeper):
    """Two large removable half rings; each retains two existing M3 holes."""
    halves={}
    for name,y in (("keeper_positive",.2),("keeper_negative",-100.)):
        length=99.8
        clip=cq.Solid.makeBox(200,length,20,cq.Vector(-100,y,10))
        halves[name]=keeper.intersect(clip).clean()
    return halves

def parts(angle=0.,travel=0.,*,connected=False,mouth=MOUTH):
    if not math.isfinite(mouth) or not 67<=mouth<=70:
        raise ValueError('unsupported mouth radius; preserve socket back and journal clearance')
    c=Core()
    port=Port()
    p=retained_parts(c,0.,travel)
    if not math.isfinite(angle) or abs(angle)>180:
        raise ValueError("angle outside prototype range")
    if travel<6.4 and abs(angle/45-round(angle/45))>1e-8:
        raise ValueError("nonindexed pose requires released clutch")
    # Stationary neck joins cup below the rotating journal.
    fixed_neck=box(27-mouth,mouth-55,24,18).translate(cq.Vector(0,0,0))
    p["stationary"]=p["stationary"].fuse(fixed_neck).fuse(placed(retained_socket(port,False),False,mouth=mouth)).clean()
    # Integral riser inside keeper opening; bridge passes above screw heads.
    riser=box(19,6,10,10).translate(cq.Vector(0,0,25.5))
    bridge=box(19,mouth-46,12,8).translate(cq.Vector(0,0,30.5))
    moving=p["rotating"].fuse(riser).fuse(bridge).fuse(placed(retained_socket(port,True),True,mouth=mouth)).clean()
    p["rotating"]=pose(moving,angle)
    p.update(split_keeper(p.pop("keeper")))
    for name in ("stationary","rotating"):
        if not p[name].isValid() or len(p[name].Solids())!=1:
            raise ValueError("invalid integral housing: "+name)
    if connected:
        for side,moving_side in (("fixed",False),("moving",True)):
            p[side+"_plug"]=placed(plug(port),moving_side,angle,mouth=mouth)
            p[side+"_pin"]=placed(compact_pin(port,moving_side),moving_side,angle,mouth=mouth)
            p[side+"_retainer"]=placed(retainer_screw(port,moving_side),moving_side,angle,mouth=mouth)
    return p

# The split keeper is installed before the four screws, with the bridge
# parallel to the split. Operating angle and assembly angle are distinct.
ASSEMBLY_ANGLE=0.

def keeper_insertion_audit(angle=ASSEMBLY_ANGLE,step=1.,*,mouth=MOUTH):
    """Sample both straight lateral insertion paths, including the other half.
    Return every detected collision; no tolerance relaxation.
    A clear sampled path is not a continuous-motion or physical certificate.
    """
    if not math.isfinite(step) or not 0<step<=2:
        raise ValueError("invalid insertion sampling step")
    p=parts(angle,0.,mouth=mouth)
    defects=[]
    installed=["stationary","rotating","plunger"]
    for name,sign in (("keeper_positive",1),("keeper_negative",-1)):
        count=math.ceil(80/step)
        for i in range(count,-1,-1):
            d=80*i/count
            q=p[name].translate(cq.Vector(0,sign*d,0))
            for n in installed:
                v=q.intersect(p[n]).Volume()
                if v>1e-6:
                    defects.append((name,n,d,v))
        installed.append(name)
    return defects

def assembly_parts():
    """Prescribed assembly pose; do not insert keeper at operating angles."""
    return parts(ASSEMBLY_ANGLE,0.)
