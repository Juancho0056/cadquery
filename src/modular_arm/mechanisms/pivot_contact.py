"""Mirrored pivot-contact prototype. Prescribed poses, not a force simulation."""
from dataclasses import dataclass, field
import math
import cadquery as cq
from modular_arm.config.joint import JointConfig
from modular_arm.mechanisms.dual_track import DualTrackConfig


@dataclass(frozen=True, slots=True)
class PivotContactConfig:
    joint: JointConfig = field(default_factory=JointConfig)
    flank_half_width_deg: float = 2.8
    tip_radius_mm: float = 1.25
    tip_root_offset_mm: float = 1.5
    pivot_x_mm: float = 22.
    pivot_radial_offset_mm: float = 4.
    boss_radius_mm: float = 4.5
    pivot_diameter_mm: float = 4.
    pivot_clearance_mm: float = .4
    arm_inner_offset_mm: float = 5.
    arm_outer_offset_mm: float = 10.
    stop_x_mm: float = 10.
    stop_radius_mm: float = 2.5
    stop_clearance_mm: float = .4
    full_release_deg: float = 20.
    lock_radius_mm: float = 24.
    lock_diameter_mm: float = 4.
    lock_clearance_mm: float = .4
    lock_head_diameter_mm: float = 10.
    lock_head_thickness_mm: float = 4.

    def __post_init__(self):
        for name in self.__dataclass_fields__:
            if name != "joint":
                v = getattr(self, name)
                if not math.isfinite(v) or v <= 0:
                    raise ValueError(f"invalid {name}")
        r = self.track.ring
        if 2*self.flank_half_width_deg >= r.pitch_degrees or r.tooth_count % 4:
            raise ValueError("incompatible tooth pitch")
        if self.arm_outer_offset_mm <= self.arm_inner_offset_mm:
            raise ValueError("invalid arm geometry")
        if self.pivot_bore_mm >= 2*self.boss_radius_mm:
            raise ValueError("invalid pivot bore")
        if self.full_release_deg >= 90:
            raise ValueError("invalid release")
        if self.lock_radius_mm-self.lock_bore_mm/2 <= r.bore_diameter_mm/2:
            raise ValueError("lock overlaps axle")
        if self.lock_radius_mm+self.lock_bore_mm/2 >= self.track.root_radius_mm:
            raise ValueError("lock outside rotor core")
        if self.lock_web_mm <= 0:
            raise ValueError("lock holes overlap")
        if self.lock_head_diameter_mm <= self.lock_bore_mm:
            raise ValueError("invalid lock head")
        for x,y,d in self.support_centers:
            if math.hypot(x,y)+d/2 >= self.joint.fork_outer_diameter_mm/2:
                raise ValueError("support outside fork")

    @property
    def track(self):
        return DualTrackConfig(ring=self.joint.cartridge.ring)

    @property
    def pivot_y_mm(self):
        return self.track.tip_radius_mm+self.pivot_radial_offset_mm

    @property
    def stop_y_mm(self):
        return self.track.tip_radius_mm+self.arm_inner_offset_mm-self.stop_radius_mm

    @property
    def pivot_bore_mm(self):
        return self.pivot_diameter_mm+self.pivot_clearance_mm

    @property
    def lock_bore_mm(self):
        return self.lock_diameter_mm+self.lock_clearance_mm

    @property
    def lock_web_mm(self):
        return 2*self.lock_radius_mm*math.sin(math.pi/self.track.ring.tooth_count)-self.lock_bore_mm

    @property
    def support_centers(self):
        return [(self.pivot_x_mm,self.pivot_y_mm,self.pivot_bore_mm),
                (-self.pivot_x_mm,self.pivot_y_mm,self.pivot_bore_mm),
                (self.stop_x_mm,self.stop_y_mm,2*self.stop_radius_mm+self.stop_clearance_mm),
                (-self.stop_x_mm,self.stop_y_mm,2*self.stop_radius_mm+self.stop_clearance_mm)]


def profile(c, reverse=False):
    points=[]
    t=c.track
    for i in range(t.ring.tooth_count):
        for a,r in ((-c.flank_half_width_deg,t.root_radius_mm),
                    (c.flank_half_width_deg,t.root_radius_mm),
                    (c.flank_half_width_deg,t.tip_radius_mm)):
            angle=math.radians(90+i*t.ring.pitch_degrees+a)
            x,y=r*math.cos(angle),r*math.sin(angle)
            points.append((-x if reverse else x,y))
    return points[::-1] if reverse else points


def make_contact_rotor(c=None, *, lock_holes=True):
    c=c or PivotContactConfig()
    t=c.track
    h=t.track_thickness_mm
    low=cq.Workplane("XY").polyline(profile(c)).close().extrude(h)
    high=cq.Workplane("XY").polyline(profile(c,True)).close().extrude(h).translate((0,0,h+t.track_axial_gap_mm))
    core=cq.Workplane("XY").circle(t.root_radius_mm).extrude(t.ring.thickness_mm)
    bore=cq.Workplane("XY").circle(t.ring.bore_diameter_mm/2).extrude(t.ring.thickness_mm)
    rotor=core.union(low).union(high).cut(bore).val()
    if lock_holes:
        for i in range(t.ring.tooth_count):
            a=math.radians(i*t.ring.pitch_degrees)
            rotor=rotor.cut(cq.Solid.makeCylinder(c.lock_bore_mm/2,t.ring.thickness_mm,
                cq.Vector(c.lock_radius_mm*math.cos(a),c.lock_radius_mm*math.sin(a),0)))
    return rotor


def make_contact_pawl(c=None, *, release_deg=0., reverse=False):
    c=c or PivotContactConfig()
    if not math.isfinite(release_deg) or not 0<=release_deg<=c.full_release_deg:
        raise ValueError("release outside operating range")
    t=c.track
    h=t.follower_thickness_mm
    tip_y=t.root_radius_mm+c.tip_root_offset_mm
    outer=t.tip_radius_mm+c.arm_outer_offset_mm
    inner=t.tip_radius_mm+c.arm_inner_offset_mm
    w=c.tip_radius_mm
    x,y=c.pivot_x_mm,c.pivot_y_mm
    p=cq.Workplane("XY").center(0,tip_y).circle(w).extrude(h)
    p=p.union(cq.Workplane("XY").center(0,(tip_y+outer)/2).rect(2*w,outer-tip_y).extrude(h))
    p=p.union(cq.Workplane("XY").center((x-w)/2,(inner+outer)/2).rect(x+w,outer-inner).extrude(h))
    p=p.union(cq.Workplane("XY").center(x,y).circle(c.boss_radius_mm).extrude(h))
    p=p.cut(cq.Workplane("XY").center(x,y).circle(c.pivot_bore_mm/2).extrude(h)).val()
    p=p.rotate((x,y,0),(x,y,1),-release_deg).translate(cq.Vector(0,0,t.follower_clearance_mm/2))
    if reverse:
        p=p.mirror("YZ").translate(cq.Vector(0,0,t.track_thickness_mm+t.track_axial_gap_mm))
    return p


def make_contact_stop(c=None, reverse=False):
    c=c or PivotContactConfig()
    return cq.Solid.makeCylinder(c.stop_radius_mm,c.track.ring.thickness_mm,
        cq.Vector(-c.stop_x_mm if reverse else c.stop_x_mm,c.stop_y_mm,0))
