"""Independent indexing bench; not a self-ratcheting mechanism."""
from dataclasses import dataclass, field
import math
import cadquery as cq
from modular_arm.config.ring import ExperimentalRingConfig

@dataclass(frozen=True, slots=True)
class IndexingConfig:
    ring: ExperimentalRingConfig = field(default_factory=ExperimentalRingConfig)
    slot_width_mm: float = 4.0
    slot_depth_mm: float = 5.5
    tip_width_mm: float = 2.4
    engagement_depth_mm: float = 3.0
    pawl_thickness_mm: float = 8.0
    arm_inner_offset_mm: float = 3.0
    arm_outer_offset_mm: float = 7.0
    pivot_x_mm: float = 22.0
    pivot_y_offset_mm: float = 1.0
    boss_radius_mm: float = 4.5
    pivot_diameter_mm: float = 4.0
    pivot_clearance_mm: float = 0.4
    release_degrees: float = 25.0

    @property
    def radius_mm(self):
        return self.ring.outer_diameter_mm / 2

    @property
    def pivot_y_mm(self):
        return self.radius_mm + self.pivot_y_offset_mm

    def __post_init__(self):
        values = (self.slot_width_mm, self.slot_depth_mm, self.tip_width_mm,
                  self.engagement_depth_mm, self.pawl_thickness_mm, self.arm_inner_offset_mm,
                  self.arm_outer_offset_mm, self.pivot_x_mm, self.pivot_y_offset_mm,
                  self.boss_radius_mm, self.pivot_diameter_mm, self.pivot_clearance_mm,
                  self.release_degrees)
        if not all(math.isfinite(v) and v > 0 for v in values):
            raise ValueError("positive finite dimensions required")
        if self.ring.tooth_count % 4:
            raise ValueError("opposed indexed positions require tooth count divisible by four")
        if not self.tip_width_mm < self.slot_width_mm < 2*(self.radius_mm-self.slot_depth_mm)*math.sin(math.pi/self.ring.tooth_count):
            raise ValueError("invalid slot or web width")
        if not self.engagement_depth_mm < self.slot_depth_mm < self.radius_mm-self.ring.bore_diameter_mm/2:
            raise ValueError("invalid slot depth")
        if self.pawl_thickness_mm >= self.ring.thickness_mm:
            raise ValueError("pawl must fit within rotor axial envelope")
        if self.arm_outer_offset_mm <= self.arm_inner_offset_mm or self.release_degrees >= 90:
            raise ValueError("invalid arm or release geometry")
        if self.pivot_diameter_mm+self.pivot_clearance_mm >= 2*self.boss_radius_mm:
            raise ValueError("pivot bore too large")


def make_indexed_rotor(c=None):
    c = c or IndexingConfig()
    r = c.radius_mm
    rotor = cq.Workplane('XY').circle(r).extrude(c.ring.thickness_mm).faces('>Z').workplane().hole(c.ring.bore_diameter_mm)
    slot = cq.Workplane('XY').box(c.slot_width_mm,c.slot_depth_mm+2,c.ring.thickness_mm+2,centered=(True,False,False)).translate((0,r-c.slot_depth_mm,-1))
    for i in range(c.ring.tooth_count):
        rotor = rotor.cut(slot.rotate((0,0,0),(0,0,1),i*c.ring.pitch_degrees))
    return rotor.val()


def make_upper_pawl(c=None):
    c = c or IndexingConfig()
    r = c.radius_mm
    w = c.tip_width_mm/2
    x = c.pivot_x_mm
    y = c.pivot_y_mm
    points = [(-w,r-c.engagement_depth_mm),(w,r-c.engagement_depth_mm),
              (w,r+c.arm_inner_offset_mm),(x,r+c.arm_inner_offset_mm),
              (x,r+c.arm_outer_offset_mm),(-w,r+c.arm_outer_offset_mm)]
    pawl = cq.Workplane('XY').polyline(points).close().extrude(c.pawl_thickness_mm)
    boss = cq.Workplane('XY').center(x,y).circle(c.boss_radius_mm).extrude(c.pawl_thickness_mm)
    bore = cq.Workplane('XY').center(x,y).circle((c.pivot_diameter_mm+c.pivot_clearance_mm)/2).extrude(c.pawl_thickness_mm)
    return pawl.union(boss).cut(bore).val().translate(cq.Vector(0,0,(c.ring.thickness_mm-c.pawl_thickness_mm)/2))


def posed_pawl(c, degrees, lower=False):
    if not 0 <= degrees <= c.release_degrees:
        raise ValueError("pawl angle outside experimental travel")
    p = make_upper_pawl(c).rotate((c.pivot_x_mm,c.pivot_y_mm,0),(c.pivot_x_mm,c.pivot_y_mm,1),-degrees)
    return p.mirror('XZ') if lower else p


def indexing_parts(c=None, upper_release=0., lower_release=0., rotor_angle=0.):
    c = c or IndexingConfig()
    return {'rotor':make_indexed_rotor(c).rotate((0,0,0),(0,0,1),rotor_angle),
            'upper_pawl':posed_pawl(c,upper_release), 'lower_pawl':posed_pawl(c,lower_release,True),
            'upper_pivot':cq.Solid.makeCylinder(c.pivot_diameter_mm/2,c.ring.thickness_mm,cq.Vector(c.pivot_x_mm,c.pivot_y_mm,0)),
            'lower_pivot':cq.Solid.makeCylinder(c.pivot_diameter_mm/2,c.ring.thickness_mm,cq.Vector(c.pivot_x_mm,-c.pivot_y_mm,0))}
