"""Deterministic checks that do not imply structural or functional validation."""
from dataclasses import asdict
from modular_arm.config.ring import ExperimentalRingConfig
from modular_arm.components.experimental_ring import make_experimental_ring


def check_experimental_ring(config: ExperimentalRingConfig | None = None) -> dict:
    c = config or ExperimentalRingConfig()
    ring = make_experimental_ring(c)
    shape = ring.val()
    bb = shape.BoundingBox()
    expected = {"width_x_mm": c.outer_diameter_mm, "width_y_mm": c.outer_diameter_mm,
                "height_z_mm": c.thickness_mm, "pitch_degrees": c.pitch_degrees}
    actual = {"width_x_mm": bb.xlen, "width_y_mm": bb.ylen,
              "height_z_mm": bb.zlen, "pitch_degrees": 360.0 / c.tooth_count}
    checks = {
        "H01_valid": bool(shape.isValid()),
        "H01_solid_count": len(shape.Solids()) == 1,
        "H01_positive_volume": shape.Volume() > 0,
        "H02_outer_size": abs(bb.xlen - c.outer_diameter_mm) < 1e-4 and abs(bb.ylen - c.outer_diameter_mm) < 1e-4,
        "H02_thickness": abs(bb.zlen - c.thickness_mm) < 1e-4,
        "H02_pitch": abs(c.pitch_degrees * c.tooth_count - 360.0) < 1e-9,
    }
    return {"component": "experimental_ring_v0", "status": "PASS" if all(checks.values()) else "FAIL",
            "checks": checks, "parameters": asdict(c), "expected": expected,
            "actual": actual, "volume_mm3": shape.Volume(),
            "limitations": "H03-H10 and physical load tests are NOT_RUN"}
