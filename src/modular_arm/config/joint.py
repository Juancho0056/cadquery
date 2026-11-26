"""Preliminary joint interface; all lengths in mm."""
from dataclasses import dataclass, field
import math
from .cartridge import CartridgeConfig

@dataclass(frozen=True, slots=True)
class JointConfig:
    cartridge: CartridgeConfig = field(default_factory=CartridgeConfig)
    fork_outer_diameter_mm: float = 106.0
    bridge_width_mm: float = 34.0
    bridge_depth_mm: float = 12.0
    pin_diameter_mm: float = 5.0
    pin_clearance_mm: float = 0.4
    pin_head_diameter_mm: float = 9.0
    pin_head_thickness_mm: float = 3.0
    pin_tip_extension_mm: float = 4.0
    radial_clearance_mm: float = 1.0

    def __post_init__(self):
        c = self.cartridge
        dims = (self.fork_outer_diameter_mm,self.bridge_width_mm,self.bridge_depth_mm,
                self.pin_diameter_mm,self.pin_clearance_mm,self.pin_head_diameter_mm,
                self.pin_head_thickness_mm,self.pin_tip_extension_mm,self.radial_clearance_mm)
        if not all(math.isfinite(v) and v > 0 for v in dims):
            raise ValueError("positive finite dimensions required")
        if self.pin_head_diameter_mm <= self.pin_bore_mm:
            raise ValueError("pin head too small")
        if self.pin_bore_mm >= c.retention_extension_mm or self.pin_bore_mm >= c.axle_diameter_mm:
            raise ValueError("pin bore too large")
        if self.pin_tip_extension_mm >= c.retention_extension_mm:
            raise ValueError("invalid pin tip extension")
        if self.bridge_inner_x_mm >= self.fork_outer_diameter_mm/2:
            raise ValueError("bridge must overlap fork plates")
        if self.bridge_width_mm/2 >= math.sqrt((self.fork_outer_diameter_mm/2)**2-self.bridge_inner_x_mm**2):
            raise ValueError("bridge width exceeds plate overlap")

    @property
    def pin_bore_mm(self):
        return self.pin_diameter_mm + self.pin_clearance_mm

    @property
    def stack_height_mm(self):
        c = self.cartridge
        return 2*c.fork_wall_mm+c.fork_gap_mm

    @property
    def pin_center_z_mm(self):
        c = self.cartridge
        return c.head_thickness_mm+self.stack_height_mm+c.retention_extension_mm/2

    @property
    def bridge_inner_x_mm(self):
        return self.cartridge.ring.outer_diameter_mm/2+self.radial_clearance_mm

    @property
    def pin_shaft_length_mm(self):
        return self.cartridge.axle_diameter_mm+self.pin_tip_extension_mm
