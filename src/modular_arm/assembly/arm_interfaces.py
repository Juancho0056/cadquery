"""Limited-travel integral arm interfaces, experimental and not load rated."""
from dataclasses import dataclass
import math
import cadquery as cq

@dataclass(frozen=True,slots=True)
class ArmInterfaceConfig:
    reach_mm: float=120.
    fixed_width_mm: float=24.
    moving_width_mm: float=18.
    thickness_mm: float=18.
    moving_root_radius_mm: float=28.
    bridge_overlap_mm: float=2.
    mounting_hole_mm: float=6.4
    mounting_centers_mm: tuple[float,float]=(85.,105.)
    sweep_limit_deg: float=30.

    def __post_init__(self):
        values=(self.reach_mm,self.fixed_width_mm,self.moving_width_mm,self.thickness_mm,
                self.moving_root_radius_mm,self.bridge_overlap_mm,self.mounting_hole_mm,self.sweep_limit_deg,
                *self.mounting_centers_mm)
        if not all(math.isfinite(v) and v>0 for v in values):
            raise ValueError("positive finite arm dimensions required")
        if len(self.mounting_centers_mm)!=2 or not self.mounting_centers_mm[0]<self.mounting_centers_mm[1]:
            raise ValueError("two ordered mounting centers required")
        if self.mounting_hole_mm>=min(self.fixed_width_mm,self.moving_width_mm):
            raise ValueError("hole exceeds arm width")
        if self.mounting_centers_mm[1]+self.mounting_hole_mm/2>=self.reach_mm:
            raise ValueError("hole outside arm")
        if self.mounting_centers_mm[1]-self.mounting_centers_mm[0]<=self.mounting_hole_mm:
            raise ValueError("mounting holes overlap")
        if self.sweep_limit_deg>=90:
            raise ValueError("limited travel only")


def interface_solids(c,a=None):
    a=a or ArmInterfaceConfig()
    j=c.contact.joint
    k=j.cartridge
    z0=k.head_thickness_mm+k.fork_wall_mm+k.axial_clearance_per_side_mm
    z=z0+(k.ring.thickness_mm-a.thickness_mm)/2
    start=j.bridge_inner_x_mm+j.bridge_depth_mm-a.bridge_overlap_mm
    if not 0<a.bridge_overlap_mm<j.bridge_depth_mm:
        raise ValueError("invalid bridge overlap")
    if a.thickness_mm>=k.ring.thickness_mm or a.fixed_width_mm>j.bridge_width_mm:
        raise ValueError("arm outside axial gap or bridge width")
    if not c.contact.lock_radius_mm+c.contact.lock_bore_mm/2<a.moving_root_radius_mm< c.contact.track.root_radius_mm:
        raise ValueError("moving root must overlap core without blocking lock holes")
    if not max(start,a.moving_root_radius_mm,j.fork_outer_diameter_mm/2)<a.mounting_centers_mm[0]-a.mounting_hole_mm/2:
        raise ValueError("mounting holes must be outside cartridge")
    fixed=(cq.Workplane("XY").box(a.reach_mm-start,a.fixed_width_mm,a.thickness_mm,
                centered=(False,True,False)).translate((start,0,z)).val())
    moving=(cq.Workplane("XY").box(a.moving_width_mm,a.reach_mm-a.moving_root_radius_mm,a.thickness_mm,
                centered=(True,False,False)).translate((0,-a.reach_mm,z)).val())
    for distance in a.mounting_centers_mm:
        for name,x,y in (("fixed",distance,0),("moving",0,-distance)):
            bore=cq.Solid.makeCylinder(a.mounting_hole_mm/2,a.thickness_mm+2,cq.Vector(x,y,z-1))
            if name=="fixed":
                fixed=fixed.cut(bore)
            else:
                moving=moving.cut(bore)
    return fixed,moving


def attach_arm_interfaces(c,parts,a=None):
    """Fuse load interfaces into existing structural solids, not actuator shafts."""
    fixed,moving=interface_solids(c,a)
    for name,arm in (("housing",fixed),("rotor",moving)):
        if parts[name].intersect(arm).Volume()<=1e-6:
            raise ValueError(f"{name}: arm has no structural overlap")
        joined=parts[name].fuse(arm)
        if not joined.isValid() or len(joined.Solids())!=1:
            raise ValueError(f"{name}: disconnected interface")
        parts[name]=joined
    return parts
