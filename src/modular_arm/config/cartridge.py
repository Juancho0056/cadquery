"""Preliminary interface dimensions, in mm; not a load rating."""
from dataclasses import dataclass, field
import math
from .ring import ExperimentalRingConfig


@dataclass(frozen=True, slots=True)
class CartridgeConfig:
    ring: ExperimentalRingConfig = field(default_factory=ExperimentalRingConfig)
    axle_diameter_mm: float = 20.0
    fork_wall_mm: float = 15.0
    axial_clearance_per_side_mm: float = 0.4
    bearing_diametral_clearance_mm: float = 0.6
    head_diameter_mm: float = 30.0
    head_thickness_mm: float = 5.0
    retention_extension_mm: float = 12.0

    def __post_init__(self):
        dimensions = (self.axle_diameter_mm, self.fork_wall_mm,
                      self.axial_clearance_per_side_mm,
                      self.bearing_diametral_clearance_mm,
                      self.head_diameter_mm, self.head_thickness_mm,
                      self.retention_extension_mm)
        if not all(math.isfinite(v) and v > 0 for v in dimensions):
            raise ValueError("positive finite interface dimensions required")
        if self.ring.bore_diameter_mm <= self.axle_diameter_mm:
            raise ValueError("ring must have running clearance")
        if self.head_diameter_mm <= self.bearing_bore_mm:
            raise ValueError("head must exceed bearing bore")

    @property
    def bearing_bore_mm(self):
        return self.axle_diameter_mm + self.bearing_diametral_clearance_mm

    @property
    def fork_gap_mm(self):
        return self.ring.thickness_mm + 2 * self.axial_clearance_per_side_mm

    @property
    def support_span_mm(self):
        return self.fork_gap_mm + self.fork_wall_mm

    @property
    def shaft_length_mm(self):
        return self.fork_gap_mm + 2 * self.fork_wall_mm + self.retention_extension_mm

    @property
    def total_axle_length_mm(self):
        return self.shaft_length_mm + self.head_thickness_mm
