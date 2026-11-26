"""Independent experimental fork and transverse axle stop.

The pin blocks axial escape while inserted. Its own anti-loss latch is
NOT implemented; no load-bearing use is permitted.
"""
import cadquery as cq
from modular_arm.config.joint import JointConfig
from modular_arm.components.axle import make_axle


def make_fork(config: JointConfig | None = None):
    j = config or JointConfig()
    c = j.cartridge
    r = j.fork_outer_diameter_mm/2
    lower = cq.Workplane("XY").circle(r).extrude(c.fork_wall_mm)
    upper = lower.translate((0,0,c.fork_wall_mm+c.fork_gap_mm))
    bridge = (cq.Workplane("XY").box(j.bridge_depth_mm,j.bridge_width_mm,j.stack_height_mm,
                centered=(False,True,False)).translate((j.bridge_inner_x_mm,0,0)))
    fork = lower.union(upper).union(bridge)
    bore = cq.Workplane("XY").circle(c.bearing_bore_mm/2).extrude(j.stack_height_mm)
    return fork.cut(bore)


def make_retained_axle(config: JointConfig | None = None):
    j = config or JointConfig()
    c = j.cartridge
    # Cross-hole only in the extension outside the support stack.
    hole = cq.Solid.makeCylinder(j.pin_bore_mm/2,c.axle_diameter_mm+2,
        cq.Vector(-c.axle_diameter_mm/2-1,0,j.pin_center_z_mm),cq.Vector(1,0,0))
    return make_axle(c).cut(hole)


def make_cross_pin(config: JointConfig | None = None):
    j = config or JointConfig()
    head = cq.Workplane("XY").circle(j.pin_head_diameter_mm/2).extrude(j.pin_head_thickness_mm)
    return head.faces(">Z").workplane().circle(j.pin_diameter_mm/2).extrude(j.pin_shaft_length_mm)
