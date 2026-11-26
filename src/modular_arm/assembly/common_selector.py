"""Integrated translating selector with unilateral eccentric drive.

Rigid nominal geometry only. Spring action, contact forces and load capacity
are not established by prescribed poses.
"""
from dataclasses import dataclass, field
import math
import cadquery as cq
from modular_arm.assembly.manual_actuation import ActuationConfig, manual_parts
from modular_arm.mechanisms.selector_contract import Mode
from modular_arm.mechanisms.installed_return import ReturnConfig, return_leaf


@dataclass(frozen=True, slots=True)
class CommonSelectorConfig:
    actuation: ActuationConfig = field(default_factory=ActuationConfig)
    return_config: ReturnConfig = field(default_factory=ReturnConfig)
    return_radius_mm: float = 12.0
    return_post_radius_mm: float = 1.5
    return_retainer_diameter_mm: float = 2.0
    return_retainer_clearance_mm: float = 0.3
    return_retainer_gap_mm: float = 0.3
    lever_radius_mm: float = 16.0
    follower_radius_mm: float = 2.0
    lever_arm_width_mm: float = 2.4
    contact_edge_margin_mm: float = 0.1
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
    index_pin_diameter_mm: float = 2.5
    index_pin_clearance_mm: float = 0.4
    index_head_diameter_mm: float = 6.0
    index_head_height_mm: float = 3.0

    def __post_init__(self):
        for name in self.__dataclass_fields__:
            if name not in ("actuation","return_config"):
                v = getattr(self, name)
                if not math.isfinite(v) or v <= 0:
                    raise ValueError(name)
        if self.index_pin_diameter_mm+self.index_pin_clearance_mm >= min(self.stroke_mm,self.slider_width_mm):
            raise ValueError("selector index holes overlap or exceed crossbar")
        if self.index_head_diameter_mm <= self.index_pin_diameter_mm+self.index_pin_clearance_mm:
            raise ValueError("index head must exceed bore")
        if self.lever_arm_width_mm >= 2*self.follower_radius_mm:
            raise ValueError("lever arm must be narrower than follower")
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
    arm=box_at(x,y+c.lever_radius_mm/2,z,c.lever_arm_width_mm,c.lever_radius_mm,c.slider_height_mm)
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
    low_y=c.contact.pivot_y_mm+c.lever_radius_mm*math.cos(math.radians(c.contact.full_release_deg))-c.contact_edge_margin_mm
    high_y=c.follower_y_mm+c.pad_length_mm+c.slider_width_mm/2
    y=(low_y+high_y)/2
    # Crossbar is outside the swept lever tips. Pushers are on the inner sides.
    cross=box_at(0,c.follower_y_mm+c.pad_length_mm,z,2*c.half_span_mm,c.slider_width_mm,c.slider_height_mm)
    for sign in (-1,1):
        x=sign*(c.half_span_mm-c.follower_radius_mm-c.neutral_gap_mm-c.pusher_width_mm/2)
        pad=box_at(x,y,z,c.pusher_width_mm,high_y-low_y,c.slider_height_mm)
        cross=cross.fuse(pad)
    for x in (-c.stroke_mm,0.,c.stroke_mm):
        cross=cross.cut(cq.Solid.makeCylinder((c.index_pin_diameter_mm+c.index_pin_clearance_mm)/2,
            c.slider_height_mm,cq.Vector(x,c.follower_y_mm+c.pad_length_mm,z)))
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


def index_pin(c):
    y=c.follower_y_mm+c.pad_length_mm
    z=c.drive_z_mm
    length=c.slider_height_mm+c.rail_clearance_mm+c.rail_height_mm
    shaft=cq.Solid.makeCylinder(c.index_pin_diameter_mm/2,length,cq.Vector(0,y,z))
    head=cq.Solid.makeCylinder(c.index_head_diameter_mm/2,c.index_head_height_mm,cq.Vector(0,y,z+length))
    return shaft.fuse(head)


def common_selector_parts(c=None,mode=Mode.B,*,index_inserted=True,include_arms=False,lock_inserted=True,rotor_angle=0.):
    c=c or CommonSelectorConfig()
    if not math.isfinite(rotor_angle):
        raise ValueError('nonfinite rotor angle')
    if lock_inserted and abs(rotor_angle/c.contact.track.ring.pitch_degrees-round(rotor_angle/c.contact.track.ring.pitch_degrees))>1e-8:
        raise ValueError('angular lock requires indexed rotor position')
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
    # Through hole in roof only; the pin indexes the translating crossbar.
    roof_z=c.drive_z_mm+c.slider_height_mm+c.rail_clearance_mm
    parts["lid"]=parts["lid"].cut(cq.Solid.makeCylinder(
        (c.index_pin_diameter_mm+c.index_pin_clearance_mm)/2,c.rail_height_mm+1.,
        cq.Vector(0,c.follower_y_mm+c.pad_length_mm,roof_z)))
    parts["common_selector"]=slider(c,slider_displacement(c,mode))
    if index_inserted:
        parts["selector_index_pin"]=index_pin(c)
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
    for name,shape in (keeper_parts(k).items() if lock_inserted else ()):
        parts["angular_lock_"+name]=shape.rotate((0,0,0),(1,0,0),180).translate(
            cq.Vector(-p.lock_radius_mm,0,c.top_z_mm+p.lock_head_thickness_mm))
    install_returns(c,parts,mode)
    if include_arms:
        from modular_arm.assembly.arm_interfaces import attach_arm_interfaces
        attach_arm_interfaces(c,parts)
    if rotor_angle:
        parts["rotor"]=parts["rotor"].rotate((0,0,0),(0,0,1),rotor_angle)
    return parts


def return_components(c, reverse, angle):
    """Prescribed installed shape with nominal tangent contact at free end."""
    r=c.return_config
    z=c.drive_z_mm+c.slider_height_mm+c.rail_clearance_mm
    x=c.half_span_mm
    y=c.contact.pivot_y_mm
    radius=c.return_radius_mm
    q=math.radians(angle)
    root_x=x+c.return_post_radius_mm+r.width_mm/2-r.preload_mm
    root_y=y+radius-r.length_mm
    d=r.preload_mm+radius*math.sin(q)
    # Shorten projected Y span as follower moves along its circular path.
    # This is a geometric approximation, not an elastic strain solution.
    from dataclasses import replace
    posed=replace(r,length_mm=r.length_mm-radius*(1-math.cos(q)))
    leaf=return_leaf(posed,deflection=d).translate(cq.Vector(root_x,root_y,z))
    post=cq.Solid.makeCylinder(c.return_post_radius_mm,
        c.rail_clearance_mm+r.height_mm+.2,
        cq.Vector(x+radius*math.sin(q),y+radius*math.cos(q),
                  c.drive_z_mm+c.slider_height_mm-.2))
    # Open-top socket, assembled vertically; root remains replaceable.
    width=4*r.width_mm
    extra=c.return_retainer_gap_mm+c.return_retainer_diameter_mm+1.5
    socket=box_at(root_x,root_y-r.anchor_length_mm/2,c.top_z_mm,
        width+4.,r.anchor_length_mm+4.,z+r.height_mm+extra-c.top_z_mm)
    pocket=box_at(root_x,root_y-r.anchor_length_mm/2,z,
        width+r.mounting_clearance_mm,r.anchor_length_mm+r.mounting_clearance_mm,r.height_mm+extra+1.)
    socket=socket.cut(pocket)
    # Open the exit of the leaf, preserving side walls of the root seat.
    exit_slot=box_at(root_x,root_y+1.,z,r.width_mm+r.mounting_clearance_mm,4.,r.height_mm+extra+1.)
    socket=socket.cut(exit_slot)
    pin_z=z+r.height_mm+c.return_retainer_gap_mm+c.return_retainer_diameter_mm/2
    socket=socket.cut(cq.Solid.makeCylinder((c.return_retainer_diameter_mm+c.return_retainer_clearance_mm)/2,
        width+6.,cq.Vector(root_x-width/2-3.,root_y-r.anchor_length_mm/2,pin_z),cq.Vector(1,0,0)))
    if reverse:
        return tuple(p.mirror("YZ") for p in (leaf,post,socket))
    return leaf,post,socket


def return_retainer_parts(c,reverse=False):
    from modular_arm.mechanisms.retention import KeeperConfig,keeper_parts
    r=c.return_config
    width=4*r.width_mm+4.
    x=c.half_span_mm+c.return_post_radius_mm+r.width_mm/2-r.preload_mm
    y=c.contact.pivot_y_mm+c.return_radius_mm-r.length_mm-r.anchor_length_mm/2
    z=c.drive_z_mm+c.slider_height_mm+c.rail_clearance_mm+r.height_mm+c.return_retainer_gap_mm+c.return_retainer_diameter_mm/2
    d=c.return_retainer_diameter_mm
    k=KeeperConfig(pin_diameter_mm=d,head_diameter_mm=2*d,head_thickness_mm=1.5,
        shaft_length_mm=width+3.,groove_center_mm=1.5+width+1.5,
        groove_diameter_mm=.7*d,groove_width_mm=1.2,clip_inner_diameter_mm=.8*d,
        clip_outer_diameter_mm=2*d,clip_thickness_mm=.8,clip_mouth_mm=.6*d)
    parts={}
    for name,p in keeper_parts(k).items():
        p=p.rotate((0,0,0),(0,1,0),90).translate(cq.Vector(x-width/2-1.5,y,z))
        parts[name]=p.mirror("YZ") if reverse else p
    return parts


def install_returns(c,parts,mode):
    q=c.contact.full_release_deg
    for name,reverse,angle in (("lower",False,q if mode is Mode.A else 0.),
                               ("upper",True,q if mode is Mode.C else 0.)):
        leaf,post,socket=return_components(c,reverse,angle)
        parts[name+"_actuator"]=parts[name+"_actuator"].fuse(post)
        parts["lid"]=parts["lid"].fuse(socket)
        parts[name+"_return_leaf"]=leaf
        for key,shape in return_retainer_parts(c,reverse).items():
            parts[name+"_return_retainer_"+key]=shape
