"""Universal rectangular keyed socket with transverse removable positive pin.
Coordinates: insertion along X, pin along Z. All dimensions mm.
Experimental rigid geometry, not load rated.
"""
from dataclasses import dataclass
import math
import cadquery as cq

@dataclass(frozen=True, slots=True)
class Port:
    width: float=20.
    height: float=16.
    engagement: float=24.
    wall: float=4.
    clearance: float=.4
    back: float=5.
    pin: float=5.
    pin_clearance: float=.4
    pin_center: float=12.

    def __post_init__(self):
        if any(not math.isfinite(v) or v<=0 for v in (getattr(self,n) for n in self.__dataclass_fields__)):
            raise ValueError("positive finite dimensions required")
        if self.pin+self.pin_clearance>=min(self.width,self.height) or not self.pin<self.pin_center<self.engagement-self.pin:
            raise ValueError("invalid pin location")
        if self.clearance>=self.wall or self.pin_clearance>=self.pin:
            raise ValueError("invalid clearance")

    @property
    def outer_width(self): return self.width+self.clearance+2*self.wall
    @property
    def outer_height(self): return self.height+self.clearance+2*self.wall

def box(x0,length,w,h):
    return cq.Workplane("XY").box(length,w,h,centered=(False,True,True)).translate((x0,0,0)).val()

def bore(c):
    return cq.Solid.makeCylinder((c.pin+c.pin_clearance)/2,c.outer_height+2,
        cq.Vector(-c.pin_center,0,-c.outer_height/2-1))

def socket(c=None):
    c=c or Port()
    return box(-c.engagement-c.back,c.engagement+c.back,c.outer_width,c.outer_height).cut(
        box(-c.engagement-c.clearance/2,c.engagement+c.clearance/2+1,c.width+c.clearance,c.height+c.clearance)).cut(bore(c))

def plug(c=None):
    c=c or Port()
    return box(-c.engagement,c.engagement,c.width,c.height).cut(bore(c))

def locking_pin(c=None):
    c=c or Port()
    h=c.outer_height+3
    shaft=cq.Solid.makeCylinder(c.pin/2,h,cq.Vector(-c.pin_center,0,-c.outer_height/2-3))
    head=cq.Solid.makeCylinder(c.pin,3,cq.Vector(-c.pin_center,0,c.outer_height/2))
    return shaft.fuse(head)

def segment(length=100.,c=None):
    """Independent double-ended male segment; length between shoulders."""
    c=c or Port()
    if not math.isfinite(length) or length<30:
        raise ValueError("segment too short")
    body=box(0,length,c.outer_width,c.outer_height)
    # Closed rectangular beam with accessible transverse lightening pocket.
    pocket=box(6,length-12,c.outer_width-8,c.outer_height+2)
    body=body.cut(pocket)
    return body.fuse(plug(c)).fuse(plug(c).rotate((0,0,0),(0,0,1),180).translate(cq.Vector(length,0,0)))

def assembly(length=100.,c=None,withdrawal=0.,locked=True):
    c=c or Port()
    if not math.isfinite(withdrawal) or withdrawal<0:
        raise ValueError("invalid withdrawal")
    parts={"socket_left":socket(c),"segment":segment(length,c).translate(cq.Vector(withdrawal,0,0))}
    parts["socket_right"]=socket(c).rotate((0,0,0),(0,0,1),180).translate(cq.Vector(length+withdrawal,0,0))
    if locked:
        parts["pin_left"]=locking_pin(c)
        parts["pin_right"]=locking_pin(c).rotate((0,0,0),(0,0,1),180).translate(cq.Vector(length+withdrawal,0,0))
    return parts
