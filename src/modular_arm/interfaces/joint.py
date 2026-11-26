"""Interface contract for a future functional rotor; no tooth profile assumed."""
from dataclasses import dataclass
from modular_arm.config.joint import JointConfig

@dataclass(frozen=True, slots=True)
class RotorInterface:
    bore_mm: float
    envelope_diameter_mm: float
    axial_thickness_mm: float
    pitch_degrees: float
    tooth_count: int

    @classmethod
    def from_joint(cls, config: JointConfig):
        r = config.cartridge.ring
        return cls(r.bore_diameter_mm,r.outer_diameter_mm,r.thickness_mm,
                   r.pitch_degrees,r.tooth_count)
