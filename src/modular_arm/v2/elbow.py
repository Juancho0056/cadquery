"""Compact fork elbow with positive indexed cross-pin, independent universal ports.
Not a ratchet: unload and withdraw pin before repositioning. Nominal CAD only.
"""
from dataclasses import dataclass,field
import math
import cadquery as cq
from modular_arm.v2.connector import Port,socket,box

@dataclass(frozen=True,slots=True)
class Elbow:
    port: Port=field(default_factory=Port)
    diameter: float=48.
    rotor_height: float=12.
    cheek: float=5.
    gap: float=.4
    axle: float=8.
    axle_clearance: float=.4
    lock_radius: float=17.
    lock: float=5.
    lock_clearance: float=.4
    pitch: int=30
    mouth: float=54.
    def __post_init__(self):
        if any(not math.isfinite(v) or v<=0 for n in self.__dataclass_fields__ if n!="port" for v in [getattr(self,n)]):
            raise ValueError("invalid elbow dimensions")
        if self.pitch not in (30,45,60) or self.lock_radius+(self.lock+self.lock_clearance)/2>=self.diameter/2:
            raise ValueError("invalid indexing")
        if 2*self.lock_radius*math.sin(math.radians(self.pitch/2))<=self.lock+self.lock_clearance:
            raise ValueError("overlapping index holes")
        if self.mouth-self.port.engagement-self.port.back>=self.diameter/2:
            raise ValueError("disconnected socket")
    @property
    def total(self): return self.rotor_height+2*self.gap+2*self.cheek

def cylinder(x,y,z,d,h):
    return cq.Solid.makeCylinder(d/2,h,cq.Vector(x,y,z))

def fixed_half(c):
    z=c.rotor_height/2+c.gap
    top=cylinder(0,0,z,c.diameter,c.cheek)
    bottom=cylinder(0,0,-z-c.cheek,c.diameter,c.cheek)
    # Rear bridge outside rotor radius; both cheeks extend to it.
    bridge=box(-c.diameter/2-6,6,c.port.outer_width,c.total)
    tongue=box(-c.diameter/2-6,8,c.port.outer_width,c.cheek)
    body=top.fuse(bottom).fuse(bridge).fuse(tongue.translate(cq.Vector(0,0,z+c.cheek/2))).fuse(tongue.translate(cq.Vector(0,0,-z-c.cheek/2)))
    # Mouth opens toward -X; keep cavity outside the circular joint.
    port=socket(c.port).rotate((0,0,0),(0,0,1),180).translate(cq.Vector(-c.mouth,0,0))
    body=body.fuse(port)
    for x,d in ((0,c.axle+c.axle_clearance),(c.lock_radius,c.lock+c.lock_clearance)):
        body=body.cut(cylinder(x,0,-c.total/2-1,d,c.total+2))
    return body

def moving_half(c):
    body=cylinder(0,0,-c.rotor_height/2,c.diameter,c.rotor_height)
    # Extend along +X; socket begins beyond cheek perimeter.
    neck=box(c.diameter/2-3, c.mouth-c.port.engagement-c.port.back-c.diameter/2+4, c.port.width,c.rotor_height)
    port=socket(c.port).translate(cq.Vector(c.mouth,0,0))
    body=body.fuse(neck).fuse(port)
    body=body.cut(cylinder(0,0,-c.total/2-1,c.axle+c.axle_clearance,c.total+2))
    for angle in range(0,360,c.pitch):
        t=math.radians(angle)
        body=body.cut(cylinder(c.lock_radius*math.cos(t),c.lock_radius*math.sin(t),-c.rotor_height/2-1,c.lock+c.lock_clearance,c.rotor_height+2))
    return body

def pin(c,d,x):
    shaft=cylinder(x,0,-c.total/2-2,d,c.total+2)
    return shaft.fuse(cylinder(x,0,c.total/2,d+4,3))

def elbow_parts(c=None,angle=60.,locked=True):
    c=c or Elbow()
    if not math.isfinite(angle) or abs(angle)>90:
        raise ValueError("angle outside prototype range")
    if locked and abs(angle/c.pitch-round(angle/c.pitch))>1e-8:
        raise ValueError("not indexed")
    parts={"fixed":fixed_half(c),"moving":moving_half(c).rotate((0,0,0),(0,0,1),angle),"axle":pin(c,c.axle,0)}
    if locked:
        parts["index_pin"]=pin(c,c.lock,c.lock_radius)
    return parts
