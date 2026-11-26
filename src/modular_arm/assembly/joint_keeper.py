"""Experimental joint with grooved cross pin and snap keeper; not load rated."""
import cadquery as cq
from modular_arm.config.joint import JointConfig
from modular_arm.assembly.joint import joint_parts
from modular_arm.mechanisms.retention import KeeperConfig, keeper_parts

def joint_with_keeper(config=None):
    j = config or JointConfig()
    c = j.cartridge
    parts = joint_parts(j)
    del parts["cross_pin"]
    for name, part in keeper_parts(KeeperConfig.from_joint(j)).items():
        parts[name] = part.rotate((0,0,0),(0,1,0),90).translate(
            cq.Vector(-c.axle_diameter_mm/2-j.pin_head_thickness_mm,0,j.pin_center_z_mm))
    return parts
