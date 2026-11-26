"""Experimental bolt-on extensions and positive travel stops.

Nominal rigid geometry; fastener threads, preload, fatigue and capacity not qualified.
"""
from dataclasses import dataclass
import math
import cadquery as cq
from modular_arm.assembly.arm_interfaces import ArmInterfaceConfig

@dataclass(frozen=True,slots=True)
class SegmentConfig:
    extension_length_mm: float=120.
    overlap_mm: float=45.
    plate_thickness_mm: float=8.
    plate_width_mm: float=24.
    bolt_diameter_mm: float=6.
    bolt_head_diameter_mm: float=10.
    bolt_head_height_mm: float=4.
    nut_height_mm: float=5.
    stop_radius_mm: float=5.
    stop_contact_radius_mm: float=46.
    stop_margin_mm: float=.15

    def __post_init__(self):
        if not all(math.isfinite(getattr(self,n)) and getattr(self,n)>0 for n in self.__dataclass_fields__):
            raise ValueError("positive finite dimensions required")
        if self.extension_length_mm<=self.overlap_mm or self.bolt_head_diameter_mm<=self.bolt_diameter_mm:
            raise ValueError("invalid segment or fastener")


def interface_z(c,a):
    k=c.contact.joint.cartridge
    return k.head_thickness_mm+k.fork_wall_mm+k.axial_clearance_per_side_mm+(k.ring.thickness_mm-a.thickness_mm)/2


def stop_solids(c,a=None,m=None):
    a=a or ArmInterfaceConfig()
    m=m or SegmentConfig()
    k=c.contact.joint.cartridge
    z=k.head_thickness_mm+k.fork_wall_mm-1.
    # At limit, cylinder tangent to moving arm side in its rotated local frame.
    lateral=a.moving_width_mm/2+m.stop_radius_mm+m.stop_margin_mm
    result={}
    for sign in (-1,1):
        angle=math.radians(sign*a.sweep_limit_deg)
        x=sign*lateral
        y=-m.stop_contact_radius_mm
        xx=x*math.cos(angle)-y*math.sin(angle)
        yy=x*math.sin(angle)+y*math.cos(angle)
        result[sign]=cq.Solid.makeCylinder(m.stop_radius_mm,k.fork_gap_mm+1.,cq.Vector(xx,yy,z))
    return result


def attach_stops(c,parts,a=None,m=None):
    for sign,stop in stop_solids(c,a,m).items():
        if parts["housing"].intersect(stop).Volume()<=1e-6:
            raise ValueError("stop must overlap lower support")
        parts["housing"]=parts["housing"].fuse(stop)
    return parts


def extension_parts(c,a=None,m=None):
    a=a or ArmInterfaceConfig()
    m=m or SegmentConfig()
    z=interface_z(c,a)
    start=a.reach_mm-m.overlap_mm
    if start<=c.contact.joint.fork_outer_diameter_mm/2 or start>=a.mounting_centers_mm[0]-a.mounting_hole_mm/2:
        raise ValueError("extension must cover mounting holes outside cartridge")
    if m.plate_width_mm<a.moving_width_mm or m.bolt_diameter_mm>=a.mounting_hole_mm:
        raise ValueError("invalid extension fastener fit")
    result={}
    for prefix,reverse in (("fixed",False),("moving",True)):
        plate=(cq.Workplane("XY").box(m.extension_length_mm,m.plate_width_mm,m.plate_thickness_mm,
              centered=(False,True,False)).translate((start,0,z+a.thickness_mm)).val())
        for i,x in enumerate(a.mounting_centers_mm):
            hole=cq.Solid.makeCylinder(a.mounting_hole_mm/2,m.plate_thickness_mm,cq.Vector(x,0,z+a.thickness_mm))
            plate=plate.cut(hole)
            shaft=cq.Solid.makeCylinder(m.bolt_diameter_mm/2,a.thickness_mm+m.plate_thickness_mm+m.nut_height_mm,
                         cq.Vector(x,0,z-m.nut_height_mm))
            head=cq.Solid.makeCylinder(m.bolt_head_diameter_mm/2,m.bolt_head_height_mm,
                         cq.Vector(x,0,z+a.thickness_mm+m.plate_thickness_mm))
            nut=(cq.Workplane("XY").center(x,0).polygon(6,m.bolt_head_diameter_mm).extrude(m.nut_height_mm)
                 .translate((0,0,z-m.nut_height_mm)).val())
            nut=nut.cut(cq.Solid.makeCylinder(m.bolt_diameter_mm/2,m.nut_height_mm,cq.Vector(x,0,z-m.nut_height_mm)))
            result[f"{prefix}_bolt_{i}"]=shaft.fuse(head)
            result[f"{prefix}_nut_{i}"]=nut
        result[prefix+"_extension"]=plate
        if reverse:
            for name in tuple(result):
                if name.startswith(prefix):
                    result[name]=result[name].rotate((0,0,0),(0,0,1),-90)
    return result


def attach_extensions(c,parts,a=None,m=None):
    parts.update(extension_parts(c,a,m))
    return parts
