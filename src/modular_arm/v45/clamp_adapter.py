"""V4.5 clamp adapter, unloaded fit prototype. Dimensions mm.
Indexed conical retention candidate; no load rating. Male D fits a female joint head,
not the free male end of a telescopic arm.
"""
import cadquery as cq
from pathlib import Path
from modular_arm.v4 import modular_connection as m
V=cq.Vector
STEM_D=27.4
STEM_LENGTH=14.
FLANGE_D=39.
FLANGE_H=6.
NECK_D=27.88
NECK_H=12.
LOCK_DEPTH=9.
LOCK_RADIUS=4.2
LOCK_PENETRATION=3.5
LOCK_ANGLES=(0,90,180,270)

def build():
    stem=cq.Solid.makeCylinder(STEM_D/2,STEM_LENGTH+.2,V(0,0,-STEM_LENGTH))
    flange=cq.Solid.makeCylinder(FLANGE_D/2,FLANGE_H,V(0,0,0))
    neck=cq.Solid.makeCone(FLANGE_D/2,NECK_D/2,NECK_H,V(0,0,FLANGE_H))
    start=FLANGE_H+NECK_H
    male=cq.Solid.makeCylinder(m.RADIUS,m.LENGTH+.2,V(0,0,start-.2))
    male=male.cut(cq.Solid.makeBox(20,40,m.LENGTH+.5,V(m.FLAT,-20,start-.25)))
    shape=stem.fuse(flange).fuse(neck).fuse(male).clean()
    # Preserve flat bearing shoulder; round the external shoulder/neck edge.
    edges=[e for e in shape.Edges() if e.geomType()=='CIRCLE' and abs(e.Center().z-FLANGE_H)<.001 and e.Length()>100]
    if edges: shape=shape.fillet(1.,edges).clean()
    # Original radial screw axis is 9 mm below socket mouth.
    for angle in LOCK_ANGLES:
        tool=cq.Solid.makeCone(LOCK_RADIUS+.4,.3,LOCK_PENETRATION+1.,V(STEM_D/2+1,0,-LOCK_DEPTH),V(-1,0,0))
        shape=shape.cut(tool.rotate((0,0,0),(0,0,1),angle))
    shape=shape.cut(cq.Solid.makeCylinder(m.PIN_RADIUS,40,V(0,-20,start+m.JOIN-m.PIN_AT),V(0,1,0))).clean()
    return m.valid(shape,"clamp adapter")

def export(folder):
    folder=Path(folder)
    folder.mkdir(parents=True,exist_ok=True)
    part=build()
    cq.exporters.export(part,str(folder/"Adaptador_V45.step"))
    printable=part.translate(V(0,0,STEM_LENGTH))
    for ext in ("stl","3mf"):
        cq.exporters.export(printable,str(folder/("Adaptador_V45."+ext)),tolerance=.02,angularTolerance=.08)
    return part
