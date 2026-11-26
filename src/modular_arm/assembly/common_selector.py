"""Integrated translating selector with unilateral eccentric drive.

Rigid nominal geometry only. Spring action, contact forces and load capacity
are not established by prescribed poses.
"""
from dataclasses import dataclass, field
import math
import cadquery as cq
from modular_arm.assembly.manual_actuation import ActuationConfig, manual_parts
from modular_arm.mechanisms.selector_contract import Mode


@dataclass(frozen=True, slots=True)
class CommonSelectorConfig:
    actuation: ActuationConfig = field(default_factory=ActuationConfig)
    lever_radius_mm: float = 16.0
    follower_radius_mm: float = 2.0
    neutral_gap_mm: float = 0.3
    rail_clearance_mm: float = 0.4
    rail_width_mm: float = 5.0
    rail_height_mm: float = 3.0
    rail_length_mm: float = 84.0
    rail_y_mm: float = 58.0
    slider_width_mm: float = 6.0
    slider_height_mm: float = 4.0
    pusher_width_mm: float = 3.0
    pad_length_mm: float = 9.0
    lock_extension_mm: float = 4.0
    end_clearance_mm: float = 0.3

    def __post_init__(self):
        for name in self.__dataclass_fields__:
            if name != "actuation":
                v = getattr(self, name)
                if not math.isfinite(v) or v <= 0:
                    raise ValueError(name)
        if self.lever_radius_mm <= self.actuation.handle_radius_mm:
            raise ValueError("lever must extend beyond handle")
        if self.rail_length_mm <= 2*self.half_span_mm + 2*self.stroke_mm:
            raise ValueError("rail cannot contain full travel")

    @property
    def contact(self):
        return self.actuation.enclosure.contact

    @property
    def half_span_mm(self):
        return self.contact.pivot_x_mm

    @property
    def stroke_mm(self):
        return self.neutral_gap_mm + self.lever_radius_mm*math.sin(math.radians(self.contact.full_release_deg))

    @property
    def top_z_mm(self):
        return self.contact.joint.cartridge.head_thickness_mm+self.actuation.enclosure.stack

    @property
    def drive_z_mm(self):
        return self.top_z_mm+self.actuation.axial_clearance_mm+self.actuation.handle_thickness_mm

    @property
    def follower_y_mm(self):
        return self.contact.pivot_y_mm+self.lever_radius_mm


def box_at(x,y,z,dx,dy,dz):
    return cq.Workplane("XY").box(dx,dy,dz,centered=(True,True,False)).translate((x,y,z)).val()


def slider_displacement(c,mode):
    if not isinstance(mode,Mode):
        raise ValueError("unknown mode")
    # Rightward motion pushes the +X lever clockwise; leftward pushes the -X lever CCW.
    return {Mode.A:c.stroke_mm,Mode.B:0.,Mode.C:-c.stroke_mm}[mode]


def lever(c,reverse,q):
    x=-c.half_span_mm if reverse else c.half_span_mm
    y=c.contact.pivot_y_mm
    z=c.drive_z_mm
    r=c.follower_radius_mm
    arm=box_at(x,y+c.lever_radius_mm/2,z,2*r,c.lever_radius_mm,c.slider_height_mm)
    tip=cq.Solid.makeCylinder(r,c.slider_height_mm,cq.Vector(x,y+c.lever_radius_mm,z))
    arm=arm.fuse(tip)
    hub=cq.Solid.makeCylinder(c.actuation.handle_radius_mm,c.slider_height_mm,cq.Vector(x,y,z))
    # Hub sits on actuator handle; bore remains open for inspection.
    part=arm.fuse(hub).cut(cq.Solid.makeCylinder(c.contact.pivot_bore_mm/2,c.slider_height_mm,cq.Vector(x,y,z)))
    return part.rotate((x,y,0),(x,y,1),q if reverse else -q)


def slider(c,displacement):
    if not math.isfinite(displacement) or abs(displacement)>c.stroke_mm+1e-8:
        raise ValueError("selector outside stroke")
    z=c.drive_z_mm
    y=c.follower_y_mm+c.pad_length_mm/2-1.5
    # Crossbar is outside the swept lever tips. Pushers are on the inner sides.
    cross=box_at(0,c.follower_y_mm+c.pad_length_mm,z,2*c.half_span_mm,c.slider_width_mm,c.slider_height_mm)
    for sign in (-1,1):
        x=sign*(c.half_span_mm-c.follower_radius_mm-c.neutral_gap_mm-c.pusher_width_mm/2)
        pad=box_at(x,y,z,c.pusher_width_mm,c.pad_length_mm+c.slider_width_mm+3.,c.slider_height_mm)
        cross=cross.fuse(pad)
    return cross.translate(cq.Vector(displacement,0,0))


def guide_parts(c):
    # Guide extension is integral with lid through the platform tongue.
    # Opposed L lips capture the crossbar vertically while allowing X travel.
    y=c.follower_y_mm+c.pad_length_mm
    z=c.top_z_mm
    bottom=box_at(0,y,z,c.rail_length_mm,c.slider_width_mm+2*(c.rail_width_mm+c.rail_clearance_mm),c.drive_z_mm-z-c.rail_clearance_mm)
    sides=[]
    for sign in (-1,1):
        wall_y=y+sign*(c.slider_width_mm/2+c.rail_clearance_mm+c.rail_width_mm/2)
        wall=box_at(0,wall_y,c.drive_z_mm-c.rail_clearance_mm,c.rail_length_mm,c.rail_width_mm,c.slider_height_mm+2*c.rail_clearance_mm+c.rail_height_mm)
        # Only the outer lip; inner lip on lever side would obstruct the pushers.
        if sign==1:
            lip=box_at(0,wall_y-c.rail_width_mm/2,c.drive_z_mm+c.slider_height_mm+c.rail_clearance_mm,c.rail_length_mm,2*c.rail_width_mm,c.rail_height_mm)
            wall=wall.fuse(lip)
            roof=box_at(0,y,c.drive_z_mm+c.slider_height_mm+c.rail_clearance_mm,c.rail_length_mm,
                        c.slider_width_mm+2*c.rail_clearance_mm+2*c.rail_width_mm,c.rail_height_mm)
            wall=wall.fuse(roof)
        sides.append(wall)
    # Separate wall on lever side must be relieved for full pusher sweep.
    # Stops engage crossbar ends, not the swept lever tips.
    for sign in (-1,1):
        end_x=sign*(c.half_span_mm+c.stroke_mm+c.end_clearance_mm+c.rail_width_mm/2)
        stop=box_at(end_x,y,c.drive_z_mm-c.rail_clearance_mm,c.rail_width_mm,
                    c.slider_width_mm+2*c.rail_clearance_mm,c.slider_height_mm+c.rail_clearance_mm)
        bottom=bottom.fuse(stop)
    return bottom,sides[1]


def common_selector_parts(c=None,mode=Mode.B):
    c=c or CommonSelectorConfig()
    parts=manual_parts(c.actuation,mode)
    q=c.contact.full_release_deg
    for name,reverse,angle in (("lower",False,q if mode is Mode.A else 0.),("upper",True,q if mode is Mode.C else 0.)):
        parts[name+"_actuator"]=parts[name+"_actuator"].fuse(lever(c,reverse,angle))
    bottom,outer=guide_parts(c)
    # Mounting tongue extends from lid toward guide platform.
    tongue=box_at(0,(c.contact.pivot_y_mm+c.follower_y_mm+c.pad_length_mm)/2,c.top_z_mm-3.,
                  12.,c.follower_y_mm+c.pad_length_mm-c.contact.pivot_y_mm+4.,3.)
    # Platform and tongue overlap by design at their shared interface.
    parts["lid"]=parts["lid"].fuse(tongue).fuse(bottom).fuse(outer)
    parts["common_selector"]=slider(c,slider_displacement(c,mode))
    from modular_arm.mechanisms.retention import KeeperConfig,keeper_parts
    p=c.contact
    a=p.joint.cartridge
    stack=c.actuation.enclosure.stack
    # Head above lid; grooved extension and clip below lower plate.
    k=KeeperConfig(pin_diameter_mm=p.lock_diameter_mm,head_diameter_mm=p.lock_head_diameter_mm,
        head_thickness_mm=p.lock_head_thickness_mm,shaft_length_mm=stack+c.lock_extension_mm,
        groove_center_mm=p.lock_head_thickness_mm+stack+c.lock_extension_mm/2,
        groove_diameter_mm=2.8,groove_width_mm=2.,clip_inner_diameter_mm=3.,
        clip_outer_diameter_mm=8.,clip_thickness_mm=1.6,clip_mouth_mm=2.2)
    del parts["angular_lock"]
    for name,shape in keeper_parts(k).items():
        parts["angular_lock_"+name]=shape.rotate((0,0,0),(1,0,0),180).translate(
            cq.Vector(-p.lock_radius_mm,0,c.top_z_mm+p.lock_head_thickness_mm))
    return parts
