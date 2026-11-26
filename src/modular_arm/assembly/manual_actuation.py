"""Integrated manually actuated pawls, an intermediate step toward a common selector.
No automatic return, detent or mechanical interlock is claimed.
"""
from dataclasses import dataclass,field
import math
import cadquery as cq
from modular_arm.assembly.serviceable_ratchet import ServiceableConfig,serviceable_parts
from modular_arm.mechanisms.selector_contract import Mode

@dataclass(frozen=True,slots=True)
class ActuationConfig:
    enclosure: ServiceableConfig=field(default_factory=ServiceableConfig)
    shaft_diameter_mm: float=7.0
    shaft_clearance_mm: float=.4
    handle_radius_mm: float=9.
    handle_thickness_mm: float=3.
    key_width_mm: float=3.
    key_depth_mm: float=2.
    axial_clearance_mm: float=.4

    def __post_init__(self):
        for n in self.__dataclass_fields__:
            if n!="enclosure" and not (math.isfinite(getattr(self,n)) and getattr(self,n)>0):
                raise ValueError(n)
        p=self.enclosure.contact
        if self.shaft_diameter_mm <= p.pivot_bore_mm or self.shaft_diameter_mm+self.shaft_clearance_mm>=2*p.boss_radius_mm:
            raise ValueError("actuator incompatible with pawl boss")
        if self.key_width_mm>=self.shaft_diameter_mm or self.key_depth_mm>=p.track.follower_thickness_mm:
            raise ValueError("key incompatible with pawl")

def release_commands(c,mode):
    if not isinstance(mode,Mode):
        raise ValueError("invalid mode")
    q=c.enclosure.contact.full_release_deg
    return {Mode.A:(q,0.),Mode.B:(0.,0.),Mode.C:(0.,q)}[mode]

def actuator_geometry(c,reverse=False,release=0.):
    """Axis at existing pivot. Key enters top face of pawl; shaft ends at that face."""
    e=c.enclosure
    p=e.contact
    a=p.joint.cartridge
    x=-p.pivot_x_mm if reverse else p.pivot_x_mm
    y=p.pivot_y_mm
    pawl_z=a.head_thickness_mm+a.fork_wall_mm+a.axial_clearance_per_side_mm+p.track.follower_clearance_mm/2
    if reverse:
        pawl_z+=p.track.track_thickness_mm+p.track.track_axial_gap_mm
    top=pawl_z+p.track.follower_thickness_mm
    lid_top=a.head_thickness_mm+e.stack
    shaft=cq.Solid.makeCylinder(c.shaft_diameter_mm/2,lid_top-top+c.axial_clearance_mm,
        cq.Vector(x,y,top))
    # Key is offset from axis to transmit torque, overlapping shaft.
    offset=(p.pivot_bore_mm/2+c.shaft_diameter_mm/2)/2
    key=cq.Workplane("XY").box(c.key_width_mm,c.key_width_mm,c.key_depth_mm+.5,
        centered=(True,True,False)).translate((x+offset,y,top-c.key_depth_mm)).val()
    handle=cq.Solid.makeCylinder(c.handle_radius_mm,c.handle_thickness_mm,
        cq.Vector(x,y,lid_top+c.axial_clearance_mm))
    # The upper shaft is bored axially for an independent fixed support pin.
    core=shaft.fuse(key).fuse(handle)
    core=core.cut(cq.Solid.makeCylinder(p.pivot_bore_mm/2,lid_top-top+c.handle_thickness_mm+c.key_depth_mm+3,
        cq.Vector(x,y,top-c.key_depth_mm-1)))
    return core.rotate((x,y,0),(x,y,1),release if reverse else -release)

def manual_parts(c=None,mode=Mode.B):
    c=c or ActuationConfig()
    e=c.enclosure
    p=e.contact
    a=p.joint.cartridge
    lower,upper=release_commands(c,mode)
    parts=serviceable_parts(e,angle_deg=0.,lower_release=lower,upper_release=upper,lock_inserted=True)
    # Only provide a geometry proposal. The lower actuator passes through the upper
    # track plane; compatibility is explicitly checked by the interference harness.
    for prefix,reverse,q in (("lower",False,lower),("upper",True,upper)):
        x=-p.pivot_x_mm if reverse else p.pivot_x_mm
        y=p.pivot_y_mm
        act=actuator_geometry(c,reverse,q)
        parts[prefix+"_pawl"]=parts[prefix+"_pawl"].cut(act)
        parts["lid"]=parts["lid"].cut(cq.Solid.makeCylinder(
            (c.shaft_diameter_mm+c.shaft_clearance_mm)/2,e.stack,
            cq.Vector(x,y,a.head_thickness_mm+e.split_z)))
        parts[prefix+"_actuator"]=act
    return parts
