"""Central axial dog clutch. Press to release; spring return is a design intent.
Rigid prescribed poses, not a qualified mechanism. Dimensions in mm.
"""
from dataclasses import dataclass, field
import math
import cadquery as cq
from modular_arm.v2.connector import Port, socket, box

@dataclass(frozen=True, slots=True)
class Joint:
    port: Port=field(default_factory=Port)
    diameter: float=48.
    pitch: int=30
    core_radius: float=8.
    tooth_radius: float=12.
    tooth_width: float=3.
    clearance: float=.3
    stroke: float=7.
    mouth: float=54.
    def __post_init__(self):
        if not all(math.isfinite(getattr(self,n)) and getattr(self,n)>0 for n in self.__dataclass_fields__ if n!="port"):
            raise ValueError("positive finite dimensions required")
        # Only this axial stack is supported until the full stack is parameterized.
        if self.pitch not in (30,45,60) or self.stroke!=7 or self.diameter!=48 or self.mouth!=54:
            raise ValueError("unsupported axial stack")
        if not 6.5<self.core_radius<self.tooth_radius<13 or self.tooth_width+self.clearance>=2*self.core_radius*math.sin(math.radians(self.pitch/2)):
            raise ValueError("invalid dog profile")

def cyl(r,z,h):
    return cq.Solid.makeCylinder(r,h,cq.Vector(0,0,z))

def dog(c,z,h,clearance=0.):
    p=cyl(c.core_radius+clearance,z,h)
    for a in range(0,360,c.pitch):
        tooth=box(c.core_radius-1,c.tooth_radius-c.core_radius+1+clearance,
                  c.tooth_width+2*clearance,h).translate(cq.Vector(0,0,z+h/2))
        p=p.fuse(tooth.rotate((0,0,0),(0,0,1),a))
    return p.clean()

def fork(c):
    # Lower fixed guide and upper cheek, connected by rear bridge.
    lower=cyl(24,0,14).cut(dog(c,-1,16,c.clearance))
    upper=cyl(24,22.8,5).cut(cyl(6.3,22,7))
    bridge=box(-30,6,28.4,27.8).translate(cq.Vector(0,0,13.9))
    for z,h in ((0,14),(22.8,5)):
        bridge=bridge.fuse(box(-30,8,28.4,h).translate(cq.Vector(0,0,z+h/2)))
    p=lower.fuse(upper).fuse(bridge)
    p=p.fuse(socket(c.port).rotate((0,0,0),(0,0,1),180).translate(cq.Vector(-c.mouth,0,13.9)))
    # Annular journal projects into lower rotor counterbore.
    journal=cyl(15.5,14,1.4).cut(cyl(13.1,13.9,1.6))
    return p.fuse(journal).clean()

def rotor(c):
    p=cyl(24,14.4,8)
    p=p.fuse(box(21,5,20,8).translate(cq.Vector(0,0,18.4)))
    p=p.fuse(socket(c.port).translate(cq.Vector(c.mouth,0,18.4)))
    p=p.cut(dog(c,14,9,c.clearance)).cut(cyl(15.8,14.3,1.3))
    return p.clean()

def slider(c,pressed=False):
    # Lower dogs always remain within fixed guide. Pressing moves dogs below rotor.
    p=dog(c,8,12.4).fuse(cyl(6,20.3,17.5)).clean()
    return p.translate(cq.Vector(0,0,-c.stroke if pressed else 0))

def bottom_cap(c):
    # Separate closure, nominal face fit only; fastening still to be designed.
    return cyl(16,-8,2)

def spring_envelope(c,pressed=False):
    # Reserved volume, NOT a physical spring or interference-bearing solid.
    return cyl(5,-6,7 if pressed else 14)

def parts(c=None,angle=60.,pressed=False):
    c=c or Joint()
    if not math.isfinite(angle) or abs(angle)>90:
        raise ValueError("angle outside inspection range")
    if not pressed and abs(angle/c.pitch-round(angle/c.pitch))>1e-8:
        raise ValueError("release at indexed position only")
    return {"fork":fork(c),"rotor":rotor(c).rotate((0,0,0),(0,0,1),angle),
            "button_dog":slider(c,pressed),"bottom_cap":bottom_cap(c)}
