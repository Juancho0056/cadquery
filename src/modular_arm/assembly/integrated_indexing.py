"""Integrated static indexing prototype. NOT a reversible ratchet."""
from dataclasses import dataclass, field
import math
import cadquery as cq
from modular_arm.config.joint import JointConfig
from modular_arm.mechanisms.indexing import IndexingConfig, indexing_parts
from modular_arm.assembly.joint_keeper import joint_with_keeper
from modular_arm.components.joint import make_fork

@dataclass(frozen=True)
class IntegratedConfig:
    joint: JointConfig = field(default_factory=JointConfig)
    pivot_bore_clearance_mm: float = .4
    lock_radius_mm: float = 22.
    lock_diameter_mm: float = 5.
    lock_clearance_mm: float = .4
    lock_head_diameter_mm: float = 10.
    lock_head_thickness_mm: float = 4.

    def __post_init__(self):
        vals=(self.pivot_bore_clearance_mm,self.lock_radius_mm,self.lock_diameter_mm,
              self.lock_clearance_mm,self.lock_head_diameter_mm,self.lock_head_thickness_mm)
        if not all(math.isfinite(v) and v>0 for v in vals):
            raise ValueError("positive finite dimensions required")
        r=self.joint.cartridge.ring
        h=(self.lock_diameter_mm+self.lock_clearance_mm)/2
        if self.lock_radius_mm-h <= r.bore_diameter_mm/2 or self.lock_radius_mm+h >= r.root_diameter_mm/2:
            raise ValueError("lock holes outside rotor web")
        if 2*self.lock_radius_mm*math.sin(math.pi/r.tooth_count) <= 2*h:
            raise ValueError("lock holes overlap")
        if self.lock_head_diameter_mm<=2*h:
            raise ValueError("lock head must exceed bore")
        i=self.indexing
        if math.hypot(i.pivot_x_mm,i.pivot_y_mm)+(i.pivot_diameter_mm+self.pivot_bore_clearance_mm)/2 >= self.joint.fork_outer_diameter_mm/2:
            raise ValueError("pivot outside support plate")

    @property
    def indexing(self):
        return IndexingConfig(ring=self.joint.cartridge.ring)

    @property
    def rotor_z(self):
        c=self.joint.cartridge
        return c.head_thickness_mm+c.fork_wall_mm+c.axial_clearance_per_side_mm

    @property
    def fork_top(self):
        return self.joint.cartridge.head_thickness_mm+self.joint.stack_height_mm


def bore(x,y,z,diameter,height):
    return cq.Solid.makeCylinder(diameter/2,height,cq.Vector(x,y,z))


def make_supported_fork(c=None):
    c=c or IntegratedConfig()
    j=c.joint
    i=c.indexing
    p=make_fork(j).val()
    for y in (i.pivot_y_mm,-i.pivot_y_mm):
        p=p.cut(bore(i.pivot_x_mm,y,-1,i.pivot_diameter_mm+c.pivot_bore_clearance_mm,j.stack_height_mm+2))
    return p.cut(bore(-c.lock_radius_mm,0,-1,c.lock_diameter_mm+c.lock_clearance_mm,j.stack_height_mm+2))


def make_lock_rotor(c=None):
    c=c or IntegratedConfig()
    from modular_arm.mechanisms.indexing import make_indexed_rotor
    r=make_indexed_rotor(c.indexing)
    for n in range(c.indexing.ring.tooth_count):
        a=math.radians(n*c.indexing.ring.pitch_degrees)
        r=r.cut(bore(c.lock_radius_mm*math.cos(a),c.lock_radius_mm*math.sin(a),-1,
                     c.lock_diameter_mm+c.lock_clearance_mm,c.indexing.ring.thickness_mm+2))
    return r


def integrated_parts(c=None, *, upper_release=0., lower_release=0., rotor_angle=0., lock_inserted=True):
    c=c or IntegratedConfig()
    j=c.joint
    i=c.indexing
    a=j.cartridge
    parts=joint_with_keeper(j)
    parts["fork"]=make_supported_fork(c).translate(cq.Vector(0,0,a.head_thickness_mm))
    del parts["ring"]
    parts["rotor"]=make_lock_rotor(c).rotate((0,0,0),(0,0,1),rotor_angle).translate(cq.Vector(0,0,c.rotor_z))
    p=indexing_parts(i,upper_release,lower_release,rotor_angle)
    for name in ("upper_pawl","lower_pawl"):
        parts[name]=p[name].translate(cq.Vector(0,0,c.rotor_z))
    for name,y in (("upper_pivot",i.pivot_y_mm),("lower_pivot",-i.pivot_y_mm)):
        parts[name]=bore(i.pivot_x_mm,y,a.head_thickness_mm,i.pivot_diameter_mm,j.stack_height_mm)
    if lock_inserted:
        # Removable full-depth pin, no anti-loss latch yet.
        stem=bore(-c.lock_radius_mm,0,a.head_thickness_mm,c.lock_diameter_mm,j.stack_height_mm)
        head=bore(-c.lock_radius_mm,0,c.fork_top,c.lock_head_diameter_mm,c.lock_head_thickness_mm)
        parts["angular_lock"]=stem.fuse(head)
    return parts
