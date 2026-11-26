from itertools import combinations
import math
import pytest
import cadquery as cq
from modular_arm.config.cartridge import CartridgeConfig
from modular_arm.components.axle import make_axle
from modular_arm.components.bearing_coupon import make_bearing_coupon
from modular_arm.assembly.axle_fit import fit_parts


def test_interface_stack():
    c = CartridgeConfig()
    assert c.fork_gap_mm == pytest.approx(20.8)
    assert c.support_span_mm == pytest.approx(35.8)
    assert c.total_axle_length_mm == pytest.approx(67.8)


@pytest.mark.parametrize("kwargs", [{"axle_diameter_mm": 21}, {"head_diameter_mm": 19},
                                      {"fork_wall_mm": 0}, {"retention_extension_mm": float("nan")}])
def test_reject_invalid_interfaces(kwargs):
    with pytest.raises(ValueError):
        CartridgeConfig(**kwargs)


def test_axle_geometry():
    c = CartridgeConfig()
    shape = make_axle(c).val()
    assert shape.isValid()
    assert len(shape.Solids()) == 1
    assert shape.BoundingBox().zlen == pytest.approx(c.total_axle_length_mm, abs=1e-5)
    expected = math.pi / 4 * (c.head_diameter_mm**2 * c.head_thickness_mm
                             + c.axle_diameter_mm**2 * c.shaft_length_mm)
    assert shape.Volume() == pytest.approx(expected, rel=1e-6)


def test_coupon_open_bore():
    c = CartridgeConfig()
    shape = make_bearing_coupon(c).val()
    probe = cq.Solid.makeCylinder(c.bearing_bore_mm / 2 - .05,
                                  c.fork_wall_mm + 2, cq.Vector(0,0,-1))
    assert shape.isValid()
    assert shape.intersect(probe).Volume() < 1e-6


def test_static_fit_has_no_volume_interference():
    parts = fit_parts()
    assert len(parts) == 4
    for (a, sa), (b, sb) in combinations(parts.items(), 2):
        assert sa.intersect(sb).Volume() < 1e-6, (a,b)
