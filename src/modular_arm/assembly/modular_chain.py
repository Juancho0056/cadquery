"""Reusable compact ports and interchangeable links, indexed inspection only."""
import math
import cadquery as cq
from modular_arm.assembly.common_selector import CommonSelectorConfig,common_selector_parts
from modular_arm.assembly.arm_interfaces import ArmInterfaceConfig
from modular_arm.assembly.modular_segments import SegmentConfig,interface_z

PORT=ArmInterfaceConfig(reach_mm=82.,fixed_width_mm=18.,moving_width_mm=18.,
    mounting_centers_mm=(62.,72.))

def link_parts(c,spacing=200.):
    a=PORT
    m=SegmentConfig()
    if not math.isfinite(spacing) or spacing<2*a.reach_mm+10:
        raise ValueError("insufficient joint spacing")
    lo=a.mounting_centers_mm[0]-6
    hi=spacing-lo
    z=interface_z(c,a)
    top=z+a.thickness_mm
    plate=cq.Workplane("XY").box(24,hi-lo,8,centered=(True,False,False)).translate((0,-hi,top)).val()
    # Reinforce only free span; preserve identical mounting pads at both ends.
    start=a.reach_mm
    end=spacing-a.reach_mm
    outer=cq.Workplane("XY").box(24,end-start,18,centered=(True,False,False)).translate((0,-end,top)).val()
    inner=cq.Workplane("XY").box(18,end-start-6,18,centered=(True,False,False)).translate((0,-end+3,top+3)).val()
    plate=plate.fuse(outer).cut(inner)
    parts={}
    for i,d in enumerate((*a.mounting_centers_mm,*(spacing-x for x in a.mounting_centers_mm))):
        plate=plate.cut(cq.Solid.makeCylinder(a.mounting_hole_mm/2,8,cq.Vector(0,-d,top)))
        shaft=cq.Solid.makeCylinder(3,a.thickness_mm+8+5,cq.Vector(0,-d,z-5))
        head=cq.Solid.makeCylinder(5,4,cq.Vector(0,-d,top+8))
        nut=cq.Workplane("XY").center(0,-d).polygon(6,10).extrude(5).translate((0,0,z-5)).val()
        nut=nut.cut(cq.Solid.makeCylinder(3,5,cq.Vector(0,-d,z-5)))
        parts[f"bolt_{i}"]=shaft.fuse(head)
        parts[f"nut_{i}"]=nut
    parts["segment"]=plate
    return parts

def modular_chain_parts(c=None,*,angles=(0.,0.),spacings=None):
    c=c or CommonSelectorConfig()
    angles=tuple(angles)
    if not angles or any(not math.isfinite(a) or abs(a)>PORT.sweep_limit_deg for a in angles):
        raise ValueError("invalid joint angles")
    spacings=tuple(spacings) if spacings is not None else (200.,)*(len(angles)-1)
    if len(spacings)!=len(angles)-1:
        raise ValueError("one spacing per link required")
    result={}
    origin=cq.Vector(0,0,0)
    heading=0.
    def place(part,rotation,position):
        return part.rotate((0,0,0),(0,0,1),rotation).translate(position)
    for i,angle in enumerate(angles):
        joint=common_selector_parts(c,include_arms=True,include_stops=True,
            rotor_angle=angle,arm_interface=PORT)
        for name,part in joint.items():
            result[f"j{i+1}_{name}"]=place(part,heading,origin)
        if i<len(spacings):
            travel_heading=heading+angle
            for name,part in link_parts(c,spacings[i]).items():
                result[f"link{i+1}_{name}"]=place(part,travel_heading,origin)
            rad=math.radians(travel_heading)
            origin=origin+cq.Vector(spacings[i]*math.sin(rad),-spacings[i]*math.cos(rad),0)
            heading=travel_heading+90.
    return result
