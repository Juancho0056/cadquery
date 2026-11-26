"""Corona geométrica experimental. NO ES un trinquete mecánico certificado.

Cada diente tiene dos flancos radiales; esta geometría es deliberadamente
simétrica y NO especifica perfil de uña, retención, resistencia ni cinemática.
"""
import math
import cadquery as cq
from modular_arm.config.ring import ExperimentalRingConfig


def make_experimental_ring(config: ExperimentalRingConfig | None = None) -> cq.Workplane:
    config = config or ExperimentalRingConfig()
    root_r = config.root_diameter_mm / 2.0
    tip_r = config.outer_diameter_mm / 2.0
    half_width = math.radians(config.pitch_degrees * config.tooth_fraction_of_pitch / 2.0)
    base = cq.Workplane("XY").circle(root_r).extrude(config.thickness_mm)

    # Un sólido conectado: cada trapecio penetra ligeramente el disco raíz.
    overlap_mm = min(config.tooth_radial_height_mm * 0.10, 0.40)
    inner_r = root_r - overlap_mm
    for i in range(config.tooth_count):
        a = math.radians(i * config.pitch_degrees)
        points = [
            (r * math.cos(a + s * half_width), r * math.sin(a + s * half_width))
            for r, s in ((inner_r, -1), (tip_r * math.cos(half_width), -1),
                         (tip_r, 0), (tip_r * math.cos(half_width), 1), (inner_r, 1))
        ]
        tooth = cq.Workplane("XY").polyline(points).close().extrude(config.thickness_mm)
        base = base.union(tooth)
    return base.faces(">Z").workplane().hole(config.bore_diameter_mm)
