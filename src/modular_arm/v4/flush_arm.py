"""Straight, flush cylindrical arm ends; experimental, not load rated.

Preserve the existing mating plane and locking crown. Increase thickness
outward only: the lower shell grows down, the upper shell grows up.
"""
import cadquery as cq
from modular_arm.v4 import joint
from modular_arm.v3.locking_core import Core,cyl

RADIUS=13.94
THICKNESS=28.
LIMITS={1:(-16.,12.),5:(12.4,40.4)}
AXES={i:(lo+hi)/2 for i,(lo,hi) in LIMITS.items()}
STEM_EXTENSION=14.

def blank(index):
    lo,hi=LIMITS[index]
    # Broad straight-sided shoulder in plan, with tangent circular silhouette.
    # All material stays within the axial slab; no vertical loft is used.
    from modular_arm.v4.transition import plan
    slab=cq.Solid.makeBox(100,100,THICKNESS,cq.Vector(-40,-50,lo))
    # Limit the flat shoulder to x<=32; beyond it the arm is round.
    clip=cq.Solid.makeBox(62,100,100,cq.Vector(-30,-50,-40))
    shoulder=plan().intersect(slab).intersect(clip)
    tube=cq.Solid.makeCylinder(RADIUS,24,cq.Vector(24,0,AXES[index]),cq.Vector(1,0,0))
    # Straight centerline, constant axial limits. A ruled transition changes
    # only the corner radius and width, removing the abrupt shoulder at X32.
    from modular_arm.v4.transition import rounded_wire
    z=AXES[index]
    wires=[rounded_wire(30.,19.,14.,z,.5),
           rounded_wire(45.,RADIUS,RADIUS,z,13.90)]
    ramp=cq.Solid.makeLoft(wires,ruled=True).intersect(plan()).intersect(slab)
    result=shoulder.fuse(tube).fuse(ramp).clean()
    if index==1:
        result=joint.turn(result,180)
    if not result.isValid() or len(result.Solids())!=1:
        raise ValueError("invalid flush blank")
    return result

def finished_flush(index):
    s=joint.fixed(blank(index)) if index==1 else joint.moving(blank(index))
    if index==5:
        # Extend the original stem bore to the new outer face; tooth depth,
        # journal, groove and radial retainers remain at original heights.
        c=Core()
        s=s.cut(cyl(c.stem_radius+c.clearance,23.3,18.1)).clean()
    if not s.isValid() or len(s.Solids())!=1:
        raise ValueError("invalid flush half")
    return s

def button(travel=0.):
    # Extend stem only; preserve tooth engagement and 7 mm travel.
    c=Core()
    s=joint.button(0).fuse(cyl(c.stem_radius,34.3,14.1)).clean()
    joint.button(travel) # canonical travel validation
    return s.translate(cq.Vector(0,0,-travel))

def parts(angle=0.,travel=0.):
    p=joint.parts(angle,travel)
    p["arm_a"]=finished_flush(1)
    p["arm_b"]=joint.turn(finished_flush(5),angle)
    p["button"]=button(travel)
    return p
