"""Reference-preserving interface experiment, not an assembled telescopic arm.

The original obj_3 has a measured cylindrical journal R6 at Z20.1..34.9.
Do not cut, glue or immobilize this journal until its kinematic role is known.
"""
import math
import cadquery as cq

JOURNAL_DIAMETER=12.
JOURNAL_LENGTH=14.
RADIAL_CLEARANCE=.25

def bearing_coupon(clearance=RADIAL_CLEARANCE):
    """Open-ended journal trial sleeve, deliberately separate from V4 bodies."""
    if not math.isfinite(clearance) or not .15<=clearance<=.5:
        raise ValueError("unsupported radial clearance")
    outer=cq.Solid.makeCylinder(10.,JOURNAL_LENGTH)
    bore=cq.Solid.makeCylinder(JOURNAL_DIAMETER/2+clearance,JOURNAL_LENGTH+2,cq.Vector(0,0,-1))
    return outer.cut(bore).clean()

def journal_gauge():
    return cq.Solid.makeCylinder(JOURNAL_DIAMETER/2,JOURNAL_LENGTH)
