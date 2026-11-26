"""Headed axle coupon, local +Z from underside of head.

Retention end is intentionally unfinished: no load-bearing use.
"""
import cadquery as cq
from modular_arm.config.cartridge import CartridgeConfig


def make_axle(config: CartridgeConfig | None = None) -> cq.Workplane:
    c = config or CartridgeConfig()
    head = cq.Workplane("XY").circle(c.head_diameter_mm / 2).extrude(c.head_thickness_mm)
    return head.faces(">Z").workplane().circle(c.axle_diameter_mm / 2).extrude(c.shaft_length_mm)
