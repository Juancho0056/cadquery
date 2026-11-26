"""Static assembly of experimental fork, rotor, axle and transverse stop."""
import cadquery as cq
from modular_arm.config.joint import JointConfig
from modular_arm.components.joint import make_fork, make_retained_axle, make_cross_pin
from modular_arm.components.experimental_ring import make_experimental_ring


def joint_parts(config: JointConfig | None = None):
    j = config or JointConfig()
    c = j.cartridge
    return {
        "fork": make_fork(j).val().translate(cq.Vector(0,0,c.head_thickness_mm)),
        "axle": make_retained_axle(j).val(),
        "ring": make_experimental_ring(c.ring).val().translate(
            cq.Vector(0,0,c.head_thickness_mm+c.fork_wall_mm+c.axial_clearance_per_side_mm)),
        "cross_pin": make_cross_pin(j).val().rotate((0,0,0),(0,1,0),90).translate(
            cq.Vector(-c.axle_diameter_mm/2-j.pin_head_thickness_mm,0,j.pin_center_z_mm)),
    }
