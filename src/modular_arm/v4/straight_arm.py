"""Straight-axis experimental alternative. No load or FDM qualification."""
import cadquery as cq
from modular_arm.v4 import joint

# 29 mm center spacing: nominal 1.1 mm between 27.9 mm cylinders.
# Keep the upper tube entirely above the lower disc; lower tube below rotor.
AXES={1:-2.5,5:26.5}
RADIUS=13.94

def straight_blank(index):
    if index not in AXES:
        raise ValueError("unsupported arm")
    bottom,top=(-10.,12.) if index==1 else (12.4,26.4)
    z=AXES[index]
    direction=-1 if index==1 else 1
    disc=joint.cyl(28,bottom,top-bottom)
    # Constant circular section, straight centerline. The inner end is
    # deliberately outside the functional journal (R23.35).
    tube=cq.Solid.makeCylinder(RADIUS,24,cq.Vector(direction*24,0,z),cq.Vector(direction,0,0))
    result=disc.fuse(tube).clean()
    if not result.isValid() or len(result.Solids())!=1:
        raise ValueError("straight blank disconnected")
    return result

def finished_straight(index):
    b=straight_blank(index)
    s=joint.fixed(b) if index==1 else joint.moving(b)
    if not s.isValid() or len(s.Solids())!=1:
        raise ValueError("invalid straight arm")
    return s
