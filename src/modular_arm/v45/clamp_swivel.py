"""Independent indexed swivel for V4.5. mm; unloaded fit prototype only."""
import math,json
from pathlib import Path
import cadquery as cq
from modular_arm.v45 import clamp_adapter as a
from modular_arm.v4 import modular_connection as m
V=cq.Vector
R=30.
AXLE_R=9.
RADIAL=.3
GAP=.4
PLATE_Z=22.
PLATE_H=8.
ROTOR_H=9.
INDEX_R=23.
INDEX_PIN_R=4.
INDEX_CLEAR=.3
INDEX_DEPTH=5.5
ANGLES=tuple(range(0,360,30))
ROTOR_Z=PLATE_Z+PLATE_H+GAP
D_Z=ROTOR_Z+ROTOR_H
def cyl(r,z,h,x=0.,y=0.):
    return cq.Solid.makeCylinder(r,h,V(x,y,z))
def xy(angle,r=INDEX_R):
    t=math.radians(angle)
    return r*math.cos(t),r*math.sin(t)
def fixed():
    # Preserve original clamp stem and shoulder, including its retention seats.
    b=a.build().intersect(cq.Solid.makeBox(80,80,a.STEM_LENGTH+a.FLANGE_H,V(-40,-40,-a.STEM_LENGTH)))
    b=b.fuse(cyl(R,a.FLANGE_H-.2,4.2))
    # Four supports outside the axle-retainer sweep. Open access below plate.
    for angle in (45,135,225,315):
        x,y=xy(angle,23.)
        b=b.fuse(cyl(5.,9.8,PLATE_Z-9.8+.2,x,y))
    b=b.fuse(cyl(R,PLATE_Z,PLATE_H))
    b=b.cut(cyl(AXLE_R+RADIAL,PLATE_Z-.1,PLATE_H+.2))
    for angle in ANGLES:
        x,y=xy(angle)
        b=b.cut(cyl(INDEX_PIN_R+INDEX_CLEAR,PLATE_Z+PLATE_H-INDEX_DEPTH,INDEX_DEPTH+.1,x,y))
    return m.valid(b.clean(),"swivel fixed")
def rotor():
    b=cyl(R,ROTOR_Z,ROTOR_H).fuse(cyl(AXLE_R,12.,ROTOR_Z-12.+.2))
    male=cyl(m.RADIUS,D_Z-.2,m.LENGTH+.2)
    male=male.cut(cq.Solid.makeBox(20,40,m.LENGTH+.5,V(m.FLAT,-20,D_Z-.25)))
    b=b.fuse(male)
    b=b.cut(cq.Solid.makeCylinder(m.PIN_RADIUS,40,V(0,-20,D_Z+m.JOIN-m.PIN_AT),V(0,1,0)))
    # M4 transverse axle retainer below the fixed plate.
    b=b.cut(cq.Solid.makeCylinder(m.PIN_RADIUS,40,V(0,-20,16.),V(0,1,0)))
    b=b.cut(cyl(INDEX_PIN_R+INDEX_CLEAR,ROTOR_Z-.1,ROTOR_H+.2,INDEX_R,0))
    return m.valid(b.clean(),"swivel rotor")
def index_pin():
    z=PLATE_Z+PLATE_H-INDEX_DEPTH+.3
    b=cyl(INDEX_PIN_R,z,D_Z-z+.2,INDEX_R,0).fuse(cyl(6.,D_Z,5.,INDEX_R,0))
    return m.valid(b.clean(),"index pin")
def retainer_gauge():
    # Nominal M4x25 with nut/washer envelopes; actual hardware not modeled as threads.
    shaft=cq.Solid.makeCylinder(2.,25.,V(0,-10.,16.),V(0,1,0))
    head=cq.Solid.makeCylinder(3.8,2.2,V(0,-12.2,16.),V(0,1,0))
    nut=cq.Solid.makeCylinder(4.1,4.,V(0,10.,16.),V(0,1,0))
    return shaft.fuse(head).fuse(nut).clean()
def parts(angle=0.,locked=True):
    b={"fixed":fixed(),"rotor":rotor().rotate((0,0,0),(0,0,1),angle),
       "retainer_gauge":retainer_gauge().rotate((0,0,0),(0,0,1),angle)}
    if locked: b["index_pin"]=index_pin().rotate((0,0,0),(0,0,1),angle)
    return b
def audit():
    from itertools import combinations
    checks=[]
    for angle in range(0,360,15):
        p=parts(angle,angle in ANGLES)
        for x,y in combinations(p,2):
            vol=p[x].intersect(p[y]).Volume()
            checks.append({"angle":angle,"pair":[x,y],"intersection_mm3":vol,"status":"PASS" if vol<.01 else "FAIL"})
    p=parts()
    axial=p["retainer_gauge"].translate(V(0,0,8.)).intersect(p["fixed"]).Volume()
    angular=p["index_pin"].rotate((0,0,0),(0,0,1),15.).intersect(p["fixed"]).Volume()
    checks.extend([{"pair":["axial_stop"],"intersection_mm3":axial,"status":"PASS" if axial>.01 else "FAIL"},
                   {"pair":["angular_stop"],"intersection_mm3":angular,"status":"PASS" if angular>.01 else "FAIL"}])
    return {"status":"UNLOADED_REVIEW_ONLY","sampled_checks":checks,
            "passed":all(c["status"]=="PASS" for c in checks),
            "physical_load":"NOT_RUN","retainer_hardware_fit":"NOT_RUN","pin_retention":"NOT_RUN"}
def export(folder):
    folder=Path(folder);folder.mkdir(parents=True,exist_ok=True)
    report=audit()
    (folder/"report.json").write_text(json.dumps(report,indent=2),encoding="utf-8")
    p=parts()
    assembly=cq.Assembly()
    for name,shape in p.items(): assembly.add(shape,name=name)
    assembly.save(str(folder/"REVIEW_NO_IMPRIMIR.step"))
    cq.exporters.export(cq.Compound.makeCompound(list(p.values())),str(folder/"REVIEW_NO_IMPRIMIR.3mf"))
    if not report['passed']: return report
    for name in ("fixed","rotor","index_pin"):
        shape=p[name]
        shape=shape.translate(V(0,0,-shape.BoundingBox().zmin))
        for ext in ("step","stl","3mf"):
            cq.exporters.export(shape,str(folder/(name+"."+ext)),tolerance=.03,angularTolerance=.1)
    return report
