"""Split enclosure prototype with captured support pins. Not load rated."""
from dataclasses import dataclass,field
import math
import cadquery as cq
from modular_arm.mechanisms.pivot_contact import PivotContactConfig
from modular_arm.components.joint import make_fork
from modular_arm.assembly.ratchet import ratchet_parts,cylinder
from modular_arm.mechanisms.retention import KeeperConfig,keeper_parts

@dataclass(frozen=True,slots=True)
class ServiceableConfig:
    contact: PivotContactConfig=field(default_factory=PivotContactConfig)
    socket_floor_mm: float=3.
    support_axial_clearance_mm: float=.4
    lid_pin_diameter_mm: float=6.
    lid_pin_clearance_mm: float=.4
    lid_pin_y_mm: float=10.
    lid_pin_head_mm: float=12.
    lid_pin_head_thickness_mm: float=3.
    lid_pin_extension_mm: float=4.

    @property
    def stack(self):
        return self.contact.joint.stack_height_mm

    @property
    def split_z(self):
        j=self.contact.joint
        return self.stack-j.cartridge.fork_wall_mm

    @property
    def lid_pin_x_mm(self):
        j=self.contact.joint
        return j.bridge_inner_x_mm+j.bridge_depth_mm/2

    @property
    def support_length_mm(self):
        return self.stack-2*self.socket_floor_mm-self.support_axial_clearance_mm

    def __post_init__(self):
        for n in self.__dataclass_fields__:
            if n!="contact" and (not math.isfinite(getattr(self,n)) or getattr(self,n)<=0):
                raise ValueError(n)
        j=self.contact.joint
        if self.socket_floor_mm>=j.cartridge.fork_wall_mm or self.support_length_mm<=0:
            raise ValueError("invalid blind socket")
        r=(self.lid_pin_diameter_mm+self.lid_pin_clearance_mm)/2
        if r>=j.bridge_depth_mm/2 or self.lid_pin_y_mm+r>=j.bridge_width_mm/2:
            raise ValueError("lid pin outside bridge")
        if math.hypot(self.lid_pin_x_mm,self.lid_pin_y_mm)+r>=j.fork_outer_diameter_mm/2:
            raise ValueError("lid pin outside plate")
        if self.lid_pin_head_mm<=2*r:
            raise ValueError("invalid head")


def enclosure_parts(c=None):
    c=c or ServiceableConfig()
    p=c.contact
    j=p.joint
    body=make_fork(j).val()
    for x,y,d in [(-p.lock_radius_mm,0,p.lock_bore_mm),
                  (c.lid_pin_x_mm,-c.lid_pin_y_mm,c.lid_pin_diameter_mm+c.lid_pin_clearance_mm),
                  (c.lid_pin_x_mm,c.lid_pin_y_mm,c.lid_pin_diameter_mm+c.lid_pin_clearance_mm)]:
        body=body.cut(cylinder(x,y,-1,d,c.stack+2))
    for x,y,d in p.support_centers:
        body=body.cut(cylinder(x,y,c.socket_floor_mm,d,c.stack-2*c.socket_floor_mm))
    extent=j.fork_outer_diameter_mm+2*j.bridge_depth_mm
    bottom=cq.Workplane("XY").box(extent,extent,c.split_z,centered=(True,True,False)).val()
    lid=cq.Workplane("XY").box(extent,extent,c.stack-c.split_z,centered=(True,True,False)).translate((0,0,c.split_z)).val()
    return {"housing":body.intersect(bottom),"lid":body.intersect(lid)}


def lid_keeper_config(c):
    return KeeperConfig(pin_diameter_mm=c.lid_pin_diameter_mm,head_diameter_mm=c.lid_pin_head_mm,
        head_thickness_mm=c.lid_pin_head_thickness_mm,shaft_length_mm=c.stack+c.lid_pin_extension_mm,
        groove_center_mm=c.lid_pin_head_thickness_mm+c.stack+c.lid_pin_extension_mm/2,
        groove_diameter_mm=4.5,groove_width_mm=2.,clip_inner_diameter_mm=4.7,
        clip_outer_diameter_mm=12.,clip_thickness_mm=1.6,clip_mouth_mm=3.6)


def serviceable_parts(c=None, **pose):
    c=c or ServiceableConfig()
    p=c.contact
    j=p.joint
    z=j.cartridge.head_thickness_mm
    parts=ratchet_parts(p,**pose)
    del parts["fork"]
    for name,shape in enclosure_parts(c).items():
        parts[name]=shape.translate(cq.Vector(0,0,z))
    for name,x,y,d in [("lower_pivot",p.pivot_x_mm,p.pivot_y_mm,p.pivot_diameter_mm),
                       ("upper_pivot",-p.pivot_x_mm,p.pivot_y_mm,p.pivot_diameter_mm),
                       ("lower_stop",p.stop_x_mm,p.stop_y_mm,2*p.stop_radius_mm),
                       ("upper_stop",-p.stop_x_mm,p.stop_y_mm,2*p.stop_radius_mm)]:
        parts[name]=cylinder(x,y,z+c.socket_floor_mm+c.support_axial_clearance_mm/2,d,c.support_length_mm)
    for side in (-1,1):
        for name,shape in keeper_parts(lid_keeper_config(c)).items():
            parts[f"lid_{side}_{name}"]=shape.translate(cq.Vector(c.lid_pin_x_mm,side*c.lid_pin_y_mm,z-c.lid_pin_head_thickness_mm))
    return parts
