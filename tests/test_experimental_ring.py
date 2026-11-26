import math
from modular_arm.config.ring import ExperimentalRingConfig
from modular_arm.components.experimental_ring import make_experimental_ring
from modular_arm.validation.ring_checks import check_experimental_ring


def test_h01_and_h02():
    report = check_experimental_ring()
    assert report["status"] == "PASS", report
    assert all(report["checks"].values()), report


def test_bore_is_open():
    c = ExperimentalRingConfig()
    solid = make_experimental_ring(c).val()
    # Central bore must have no material along Z through the part.
    import cadquery as cq
    probe = cq.Solid.makeCylinder(c.bore_diameter_mm / 2 - .2,
                                  c.thickness_mm + 2, cq.Vector(0,0,-1))
    assert solid.intersect(probe).Volume() < 1e-6


def test_ring_material_is_between_disc_and_envelope():
    c = ExperimentalRingConfig()
    solid = make_experimental_ring(c).val()
    disk_vol = math.pi / 4 * (c.root_diameter_mm**2 - c.bore_diameter_mm**2) * c.thickness_mm
    envelope_vol = math.pi / 4 * (c.outer_diameter_mm**2 - c.bore_diameter_mm**2) * c.thickness_mm
    assert disk_vol < solid.Volume() < envelope_vol
