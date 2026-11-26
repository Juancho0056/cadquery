"""Experimental peripheral selector cam and independent PETG spring coupons.

The cam geometry is a prototype, not a verified physical interlock.
"""
from dataclasses import dataclass
import math
import cadquery as cq
from modular_arm.mechanisms.selector_contract import Mode

@dataclass(frozen=True,slots=True)
class SelectorConfig:
    stroke_mm: float=8.
    body_width_mm: float=12.
    body_length_mm: float=42.
    body_thickness_mm: float=4.
    cam_rise_mm: float=8.
    cam_width_mm: float=4.
    channel_clearance_mm: float=.4
    spring_length_mm: float=30.
    spring_width_mm: float=6.
    spring_thickness_mm: float=1.2
    spring_anchor_mm: float=8.

    def __post_init__(self):
        if not all(math.isfinite(getattr(self,n)) and getattr(self,n)>0 for n in self.__dataclass_fields__):
            raise ValueError("positive finite dimensions required")
        if self.body_length_mm<=4*self.stroke_mm or self.body_width_mm<=2*self.cam_width_mm:
            raise ValueError("cam envelope too small")
        if self.spring_anchor_mm>=self.spring_length_mm:
            raise ValueError("invalid anchor")

def selector_position(c,mode):
    if not isinstance(mode,Mode):
        raise ValueError("unknown mode")
    return {Mode.A:-c.stroke_mm,Mode.B:0.,Mode.C:c.stroke_mm}[mode]

def cam_rises(c,x):
    """Nominal opposed ramps; no simultaneous positive command."""
    if not math.isfinite(x) or abs(x)>c.stroke_mm:
        raise ValueError("selector outside stroke")
    return (c.cam_rise_mm*max(0.,-x/c.stroke_mm),
            c.cam_rise_mm*max(0.,x/c.stroke_mm))

def make_slider(c=None):
    c=c or SelectorConfig()
    body=cq.Workplane("XY").box(c.body_length_mm,c.body_width_mm,c.body_thickness_mm,centered=(True,True,False))
    # Opposed wedge tracks, isolated axially by the base; followers not implemented.
    s=c.stroke_mm
    for sign in (-1,1):
        pts=[(0,0),(sign*s,c.cam_rise_mm),(sign*2*s,c.cam_rise_mm),(sign*2*s,0)]
        wedge=cq.Workplane("XZ").polyline(pts).close().extrude(c.cam_width_mm).translate((0,sign*c.body_width_mm/4+c.cam_width_mm/2,c.body_thickness_mm))
        body=body.union(wedge)
    return body.val()

def make_spring_coupon(c=None):
    c=c or SelectorConfig()
    # Unloaded cantilever coupon. Material response and mounting TBD.
    beam=cq.Workplane("XY").box(c.spring_length_mm,c.spring_width_mm,c.spring_thickness_mm,centered=(False,True,False))
    anchor=cq.Workplane("XY").box(c.spring_anchor_mm,2*c.spring_width_mm,2*c.spring_thickness_mm,centered=(False,True,False))
    return beam.union(anchor).val()

def selector_bench(c=None,mode=Mode.B):
    c=c or SelectorConfig()
    slider=make_slider(c).translate(cq.Vector(selector_position(c,mode),0,0))
    return {"selector_slider":slider,
            "spring_coupon_left":make_spring_coupon(c).translate(cq.Vector(-c.spring_length_mm,-2*c.body_width_mm,0)),
            "spring_coupon_right":make_spring_coupon(c).translate(cq.Vector(0,2*c.body_width_mm,0))}
