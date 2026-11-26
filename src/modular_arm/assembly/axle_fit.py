"""Virtual fit bench, not a fork or a functional ratchet."""
import cadquery as cq
from modular_arm.config.cartridge import CartridgeConfig
from modular_arm.components.axle import make_axle
from modular_arm.components.bearing_coupon import make_bearing_coupon
from modular_arm.components.experimental_ring import make_experimental_ring


def fit_parts(config: CartridgeConfig | None = None):
    c = config or CartridgeConfig()
    first = c.head_thickness_mm
    ring_z = first + c.fork_wall_mm + c.axial_clearance_per_side_mm
    second = first + c.fork_wall_mm + c.fork_gap_mm
    return {
        "axle": make_axle(c).val(),
        "bearing_lower": make_bearing_coupon(c).val().translate(cq.Vector(0, 0, first)),
        "ring": make_experimental_ring(c.ring).val().translate(cq.Vector(0, 0, ring_z)),
        "bearing_upper": make_bearing_coupon(c).val().translate(cq.Vector(0, 0, second)),
    }


def make_fit_assembly(config: CartridgeConfig | None = None):
    assembly = cq.Assembly(name="axle_fit_bench")
    for name, part in fit_parts(config).items():
        assembly.add(part, name=name)
    return assembly
