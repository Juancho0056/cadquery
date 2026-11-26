"""Separate annular test coupon for PETG running fit; not a complete fork."""
import cadquery as cq
from modular_arm.config.cartridge import CartridgeConfig


def make_bearing_coupon(config: CartridgeConfig | None = None) -> cq.Workplane:
    c = config or CartridgeConfig()
    # Use head diameter as outer diameter of the fit coupon only.
    return (cq.Workplane("XY").circle(c.head_diameter_mm / 2)
            .circle(c.bearing_bore_mm / 2).extrude(c.fork_wall_mm))
