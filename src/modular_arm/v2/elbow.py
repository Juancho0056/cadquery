"""Compact fork elbow with positive indexed cross-pin, independent universal ports.
Not a ratchet: unload and withdraw pin before repositioning. Nominal CAD only.
"""
from dataclasses import dataclass,field
import math
import cadquery as cq
from modular_arm.v2.connector import Port,socket,box
from modular_arm.mechanisms.retention import KeeperConfig,keeper_parts

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
    capture_wall: float=2.
    capture_clearance: float=.4
    capture_flange: float=7.4
    capture_flange_height: float=2.
    capture_stem: float=4.
    capture_cap_height: float=2.5
    def __post_init__(self):
        if any(not math.isfinite(v) or v<=0 for n in self.__dataclass_fields__ if n!="port" for v in [getattr(self,n)]):
            raise ValueError("invalid elbow dimensions")
        if self.pitch not in (30,45,60) or self.lock_radius+(self.lock+self.lock_clearance)/2>=self.diameter/2:
            raise ValueError("invalid indexing")
        if 2*self.lock_radius*math.sin(math.radians(self.pitch/2))<=self.lock+self.lock_clearance:
            raise ValueError("overlapping index holes")
        if not self.capture_stem<self.lock<self.capture_flange or self.capture_flange+self.capture_clearance+2*self.capture_wall>=2*(self.diameter/2-self.lock_radius):
            raise ValueError("capture guide exceeds rim")
        if self.capture_clearance>=self.cheek or self.capture_stem+self.capture_clearance>=self.capture_flange:
            raise ValueError("invalid capture geometry")
        offset=self.mouth-self.port.engagement-self.port.back-self.diameter/2
        if not 0.5<=offset<=4.:
            raise ValueError("socket back outside supported neck/bridge envelope")
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
    body=body.clean()
    if not body.isValid() or len(body.Solids())!=1:
        raise ValueError("disconnected or invalid elbow half")
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
    body=body.clean()
    if not body.isValid() or len(body.Solids())!=1:
        raise ValueError("disconnected or invalid moving half")
    return body

def pin(c,d,x):
    shaft=cylinder(x,0,-c.total/2-2,d,c.total+2)
    return shaft.fuse(cylinder(x,0,c.total/2,d+4,3))

def retained_axle(c):
    """Independent axle with head and replaceable external clip."""
    k=KeeperConfig(pin_diameter_mm=c.axle,head_diameter_mm=c.axle+4,
        head_thickness_mm=3.,shaft_length_mm=c.total+4.,
        groove_center_mm=3.+c.total+1.2,groove_diameter_mm=c.axle-1.5,
        groove_width_mm=2.,clip_inner_diameter_mm=c.axle-1.2,
        clip_outer_diameter_mm=c.axle+4.,clip_thickness_mm=1.6,
        clip_mouth_mm=c.axle-2.)
    return {name:p.rotate((0,0,0),(1,0,0),180).translate(cq.Vector(0,0,c.total/2+3.))
            for name,p in keeper_parts(k).items()}

def captive_lock(c,locked=True):
    """Pull-to-release, captive by a separate annular cap.
    No spring or fastening of cap is represented; cap must be secured before use.
    """
    x=c.lock_radius
    top=c.total/2
    stroke=c.rotor_height+c.gap+c.cheek+2.
    # Tip in locked position reaches through bottom cheek.
    bottom=-top-1.
    flange_z=top+c.capture_clearance
    flange_h=c.capture_flange_height
    cavity_top=flange_z+flange_h+stroke+c.capture_clearance
    outside=c.capture_flange+c.capture_clearance+2*c.capture_wall
    tower=cylinder(x,0,top,outside,cavity_top-top).cut(
        cylinder(x,0,top-1,c.capture_flange+c.capture_clearance,cavity_top-top+2))
    cap=cylinder(x,0,cavity_top,outside,c.capture_cap_height).cut(
        cylinder(x,0,cavity_top-1,c.capture_stem+c.capture_clearance,c.capture_cap_height+2))
    shaft=cylinder(x,0,bottom,c.lock,flange_z+flange_h-bottom)
    flange=cylinder(x,0,flange_z,c.capture_flange,flange_h)
    stem=cylinder(x,0,flange_z+flange_h,c.capture_stem,
        cavity_top+c.capture_cap_height+1-(flange_z+flange_h))
    head=cylinder(x,0,cavity_top+c.capture_cap_height+1,c.lock+4,3.)
    pull=shaft.fuse(flange).fuse(stem).fuse(head).clean()
    if not locked:
        pull=pull.translate(cq.Vector(0,0,stroke))
    return {"lock_guide":tower.clean(),"lock_cap":cap.clean(),"index_pin":pull}

def elbow_parts(c=None,angle=60.,locked=True,*,captive=False):
    c=c or Elbow()
    if not math.isfinite(angle) or abs(angle)>90:
        raise ValueError("angle outside prototype range")
    if locked and abs(angle/c.pitch-round(angle/c.pitch))>1e-8:
        raise ValueError("not indexed")
    parts={"fixed":fixed_half(c),"moving":moving_half(c).rotate((0,0,0),(0,0,1),angle)}
    axle=retained_axle(c)
    parts["axle"]=axle["grooved_pin"]
    parts["axle_keeper"]=axle["snap_keeper"]
    if captive:
        parts.update(captive_lock(c,locked))
    elif locked:
        parts["index_pin"]=pin(c,c.lock,c.lock_radius)
    return parts
