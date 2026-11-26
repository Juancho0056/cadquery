"""Hipótesis dimensionales del banco de corona, milímetros y grados."""
from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class ExperimentalRingConfig:
    outer_diameter_mm: float = 70.0
    root_diameter_mm: float = 58.0
    thickness_mm: float = 20.0
    bore_diameter_mm: float = 20.6
    tooth_count: int = 24
    # Abertura tangencial en grados por diente: ocupa 55% de cada paso angular.
    tooth_fraction_of_pitch: float = 0.55

    def __post_init__(self) -> None:
        if self.tooth_count < 3 or not isinstance(self.tooth_count, int):
            raise ValueError("tooth_count must be integer >= 3")
        if not (0 < self.bore_diameter_mm < self.root_diameter_mm < self.outer_diameter_mm):
            raise ValueError("require 0 < bore < root < outer")
        if self.thickness_mm <= 0:
            raise ValueError("thickness must be positive")
        if not (0.05 <= self.tooth_fraction_of_pitch <= 0.90):
            raise ValueError("tooth fraction outside supported geometric range")

    @property
    def pitch_degrees(self) -> float:
        return 360.0 / self.tooth_count

    @property
    def tooth_radial_height_mm(self) -> float:
        return (self.outer_diameter_mm - self.root_diameter_mm) / 2.0
