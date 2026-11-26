"""Two-joint serial arm with a shared, positively bolted lap connector.

Indexed inspection poses only; no structural rating or automatic motion claim.
"""
import math
import cadquery as cq
from modular_arm.assembly.common_selector import CommonSelectorConfig, common_selector_parts
from modular_arm.assembly.arm_interfaces import ArmInterfaceConfig
from modular_arm.assembly.modular_segments import SegmentConfig, interface_z
from modular_arm.mechanisms.selector_contract import Mode


def serial_arm_parts(c=None, *, first_angle=0., second_angle=0.):
    c=c or CommonSelectorConfig()
    a=ArmInterfaceConfig()
    m=SegmentConfig()
    for angle in (first_angle,second_angle):
        if not math.isfinite(angle) or abs(angle)>a.sweep_limit_deg:
            raise ValueError("joint outside nominal travel")
    first=common_selector_parts(c,mode=Mode.B,include_arms=True,
        include_stops=True,rotor_angle=first_angle)
    second=common_selector_parts(c,mode=Mode.B,include_arms=True,
        include_stops=True,rotor_angle=second_angle)
    # Both interfaces face each other; their distal ends are separated by 30 mm.
    spacing=2*a.reach_mm+30.
    # Connector in neutral first-joint coordinates spans both mounting patterns.
    lo=a.mounting_centers_mm[0]-10.
    hi=spacing-lo
    z=interface_z(c,a)
    top=z+a.thickness_mm
    plate=cq.Workplane("XY").box(m.plate_width_mm,hi-lo,m.plate_thickness_mm,
        centered=(True,False,False)).translate((0,-hi,top)).val()
    holes=(*a.mounting_centers_mm,*(spacing-x for x in a.mounting_centers_mm))
    fasteners={}
    for i,d in enumerate(holes):
        plate=plate.cut(cq.Solid.makeCylinder(a.mounting_hole_mm/2,m.plate_thickness_mm,cq.Vector(0,-d,top)))
        shaft=cq.Solid.makeCylinder(m.bolt_diameter_mm/2,a.thickness_mm+m.plate_thickness_mm+m.nut_height_mm,
            cq.Vector(0,-d,z-m.nut_height_mm))
        head=cq.Solid.makeCylinder(m.bolt_head_diameter_mm/2,m.bolt_head_height_mm,cq.Vector(0,-d,top+m.plate_thickness_mm))
        nut=cq.Workplane("XY").center(0,-d).polygon(6,m.bolt_head_diameter_mm).extrude(m.nut_height_mm).translate((0,0,z-m.nut_height_mm)).val()
        nut=nut.cut(cq.Solid.makeCylinder(m.bolt_diameter_mm/2,m.nut_height_mm,cq.Vector(0,-d,z-m.nut_height_mm)))
        fasteners[f"link_bolt_{i}"]=shaft.fuse(head)
        fasteners[f"link_nut_{i}"]=nut
    result={"j1_"+name:part for name,part in first.items()}
    moving={"link":plate,**fasteners}
    # Second housing +X points toward first axis in neutral coordinates.
    for name,part in second.items():
        moving["j2_"+name]=part.rotate((0,0,0),(0,0,1),90.).translate(cq.Vector(0,-spacing,0))
    for name,part in moving.items():
        result[name]=part.rotate((0,0,0),(0,0,1),first_angle)
    return result
