"""Candidate antirotation terminal geometry for original reference arms.
No thread or strength qualification. Dimensions in mm.
"""
import cadquery as cq
from modular_arm.v4 import joint
from modular_arm.v4.reference_ends import box,pin_bore,placed

def keyed_clevis():
    # Obj1 terminal width 27.90; thickness 11.50. Enclose the flat sides.
    # Leave the original cylindrical body outside the connector.
    ears=box(-12.05,-18,-11,24.1,36,29)
    cavity=box(-6.05,-14.25,-12,12.1,28.5,22.6)
    return ears.cut(cavity).cut(pin_bore()).clean()

def keyed_tongue():
    # Obj5 fork has a 11.2 gap. A broad blade extends between the ears.
    # The end stops before the fork's root; no claim of zero angular play.
    blade=box(-5.3,-11.5,-18,10.6,23,32)
    return blade.cut(pin_bore()).clean()

def parts(angle=0.,travel=0.,hardware=True):
    p=joint.parts(0.,travel,hardware=False)
    # Validate indexed operating poses using the original contract.
    joint.parts(angle,travel,hardware=False)
    p["arm_a"]=p["arm_a"].fuse(placed(keyed_clevis(),False)).clean()
    p["arm_b"]=joint.turn(p["arm_b"].fuse(placed(keyed_tongue(),True)).clean(),angle)
    if hardware:
        p.update(joint.screws(angle))
    return p
