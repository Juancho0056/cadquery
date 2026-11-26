"""Snap keeper geometry only; elastic installation and fatigue NOT_RUN."""
from dataclasses import dataclass
import math
import cadquery as cq

@dataclass(frozen=True, slots=True)
class KeeperConfig:
    pin_diameter_mm: float = 5.0
    head_diameter_mm: float = 9.0
    head_thickness_mm: float = 3.0
    shaft_length_mm: float = 24.0
    groove_center_mm: float = 25.0
    groove_diameter_mm: float = 3.5
    groove_width_mm: float = 2.0
    clip_inner_diameter_mm: float = 3.7
    clip_outer_diameter_mm: float = 10.0
    clip_thickness_mm: float = 1.6
    clip_mouth_mm: float = 2.8

    def __post_init__(self):
        if not all(math.isfinite(v) and v > 0 for v in (getattr(self,k) for k in self.__dataclass_fields__)):
            raise ValueError("positive finite dimensions required")
        if not self.clip_mouth_mm < self.groove_diameter_mm < self.clip_inner_diameter_mm < self.pin_diameter_mm < self.clip_outer_diameter_mm:
            raise ValueError("invalid radial capture")
        if not self.clip_thickness_mm < self.groove_width_mm:
            raise ValueError("clip must fit groove")
        if self.groove_center_mm-self.groove_width_mm/2 <= self.head_thickness_mm or self.groove_center_mm+self.groove_width_mm/2 >= self.head_thickness_mm+self.shaft_length_mm:
            raise ValueError("groove must be within shaft")
        if self.head_diameter_mm <= self.pin_diameter_mm:
            raise ValueError("head too small")

    @classmethod
    def from_joint(cls, j):
        return cls(pin_diameter_mm=j.pin_diameter_mm,
                   head_diameter_mm=j.pin_head_diameter_mm,
                   head_thickness_mm=j.pin_head_thickness_mm,
                   shaft_length_mm=j.pin_shaft_length_mm,
                   groove_center_mm=j.pin_head_thickness_mm+j.cartridge.axle_diameter_mm+j.pin_tip_extension_mm/2)


def keeper_parts(c=None):
    c = c or KeeperConfig()
    pin = cq.Workplane('XY').circle(c.head_diameter_mm/2).extrude(c.head_thickness_mm).faces('>Z').workplane().circle(c.pin_diameter_mm/2).extrude(c.shaft_length_mm)
    groove = cq.Workplane('XY').circle(c.pin_diameter_mm/2+1).circle(c.groove_diameter_mm/2).extrude(c.groove_width_mm).translate((0,0,c.groove_center_mm-c.groove_width_mm/2))
    z = c.groove_center_mm-c.clip_thickness_mm/2
    clip = cq.Workplane('XY').circle(c.clip_outer_diameter_mm/2).circle(c.clip_inner_diameter_mm/2).extrude(c.clip_thickness_mm).translate((0,0,z))
    mouth = cq.Workplane('XY').box(c.clip_mouth_mm,c.clip_outer_diameter_mm,c.clip_thickness_mm+2,centered=(True,False,False)).translate((0,0,z-1))
    return {'grooved_pin':pin.cut(groove).val(),'snap_keeper':clip.cut(mouth).val()}
