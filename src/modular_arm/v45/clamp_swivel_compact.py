"""Compact swivel candidate. All dimensions mm. No load qualification."""
import math,json
from pathlib import Path
from itertools import combinations
import cadquery as cq
from modular_arm.v45 import clamp_adapter as a
from modular_arm.v4 import modular_connection as m
V=cq.Vector
R=27.5
ROTOR_R=21.
JOURNAL=12.
CLEAR=.3
ROT_Z=12.4
ROT_H=8.
CAP_Z=20.8
CAP_R=23.2
CAP_TOP=29.2
D_Z=30.
PIN_Z=16.4
STROKE=5.2
ANGLES=tuple(range(0,360,30))
ENTRIES=(45,165,285)
TURN=30.
GRIP_RADIUS=1.5
GRIP_ANGLES=tuple(range(15,360,30))
TOL=.01

def cyl(r,z,h): return cq.Solid.makeCylinder(r,h,V(0,0,z))
def box(x,y,z,dx,dy,dz): return cq.Solid.makeBox(dx,dy,dz,V(x,y,z))
def radial(r,x,length,z=PIN_Z): return cq.Solid.makeCylinder(r,length,V(x,0,z),V(1,0,0))
def turn(s,a): return s.rotate((0,0,0),(0,0,1),a)
def ring(ro,ri,z,h): return cyl(ro,z,h).cut(cyl(ri,z-.1,h+.2))
def sector(a0,lo,hi,z,h,sweep):
    return turn(cq.Solid.makeCylinder(hi,h,V(0,0,z),V(0,0,1),sweep).cut(cyl(lo,z-.1,h+.2)),a0)
def fixed():
    b=a.build().intersect(box(-40,-40,-a.STEM_LENGTH,80,80,a.STEM_LENGTH+a.FLANGE_H))
    # Extend the same conical surface into the flange by 0.2 mm.
    # Avoid coincident circular base/flange edges at z=5.8 that leave
    # nonconforming tessellation boundaries. External slope unchanged.
    extension=.2
    slope=(R-a.FLANGE_D/2)/6.2
    b=b.fuse(cq.Solid.makeCone(a.FLANGE_D/2-slope*extension,R,6.2+extension,V(0,0,5.8-extension)))
    b=b.fuse(cyl(R,11.8,15.2))
    b=b.cut(cyl(JOURNAL+CLEAR,6.5,5.7))
    b=b.cut(cyl(ROTOR_R+CLEAR,12.,8.6))
    b=b.cut(cyl(CAP_R+CLEAR,20.6,7.))
    for ang in ENTRIES:
        b=b.cut(sector(ang-9,23.1,26.3,22.7,5.,18))
        b=b.cut(sector(ang-9,23.1,26.3,22.7,2.6,TURN+18))
    # Local side boss, no suspended columns.
    b=b.fuse(box(24,-5.5,12.2,11.5,11.,8.4))
    b=b.cut(radial(3.95,20.8,9.2))
    b=b.cut(radial(2.8,29.9,6.))
    # Vertical removable safety key intersects one of two pin holes.
    b=b.cut(cq.Solid.makeCylinder(1.45,10.,V(33,0,11.5)))
    return m.valid(b.clean(),"compact swivel fixed")
def rotor():
    b=cyl(ROTOR_R,ROT_Z,ROT_H).fuse(cyl(JOURNAL,7.,ROT_Z-7.+.2))
    b=b.fuse(cyl(13.5,20.2,D_Z-20.2))
    # Shoulder ends at the canonical mating plane; only the D tenon enters the socket.
    male=cyl(m.RADIUS,D_Z-.1,m.LENGTH+.1)
    male=male.cut(box(m.FLAT,-20,D_Z-.2,20,40,m.LENGTH+.4))
    b=b.fuse(male)
    b=b.cut(cq.Solid.makeCylinder(m.PIN_RADIUS,40,V(0,-20,D_Z+m.JOIN-m.PIN_AT),V(0,1,0)))
    for ang in ANGLES: b=b.cut(turn(radial(2.8,16.5,5.5),ang))
    return m.valid(b.clean(),"compact swivel rotor")
def cap():
    b=ring(CAP_R,14.,CAP_Z,6.6)
    for ang in ENTRIES: b=b.fuse(sector(ang+TURN-8,23.,26.,23.,2.,16))
    b=b.fuse(ring(R,14.,27.1,2.1))
    # Same peripheral finger scallop principle as the V4.5 elbow.
    for angle in GRIP_ANGLES:
        t=math.radians(angle)
        b=b.cut(cq.Solid.makeCylinder(GRIP_RADIUS,2.3,V(R*math.cos(t),R*math.sin(t),27.0)))
    return m.valid(b.clean(),"compact swivel cap")
def pin():
    b=radial(2.5,17.5,24.).fuse(radial(3.7,22.2,2.))
    for x in (33.,33.-STROKE):
        b=b.cut(cq.Solid.makeCylinder(1.4,8.,V(x,0,PIN_Z-4)))
    return m.valid(b.clean(),"captive pin")
def pin_key(): return cq.Solid.makeCylinder(1.15,10.,V(33,0,11.8))
def parts(angle=0.,locked=True):
    if locked and abs(angle/30-round(angle/30))>1e-8: raise ValueError("not indexed")
    return {"fixed":fixed(),"rotor":turn(rotor(),angle),"cap":cap(),
            "pin":pin().translate(V(0 if locked else STROKE,0,0)),"pin_key":pin_key()}
def overlap(a,b): return a.intersect(b).Volume()
def audit():
    rows=[]
    def record(name,v,stop=False):
        rows.append({"check":name,"intersection_mm3":v,"status":"PASS" if (v>TOL if stop else v<=TOL) else "FAIL"})
    p=parts()
    for angle in range(0,360,15):
        q=dict(p);q["rotor"]=turn(p["rotor"],angle)
        q["pin"]=p["pin"].translate(V(0 if angle%30==0 else STROKE,0,0))
        for x,y in combinations(q,2): record(f"pose {angle} {x}/{y}",overlap(q[x],q[y]))
    for dx in range(-25,1):
        record(f"pin insertion {dx}",overlap(p["pin"].translate(V(dx,0,0)),p["fixed"]))
    for lift in (20.,10.,5.,2.,1.,0.):
        record(f"rotor insertion {lift}",overlap(p["rotor"].translate(V(0,0,lift)),p["fixed"])+overlap(p["rotor"].translate(V(0,0,lift)),p["pin"].translate(V(STROKE,0,0))))
    for lift in (8.,4.,2.,1.,0.):
        for key in ("pin_key",):
            record(f"{key} insertion {lift}",sum(overlap(p[key].translate(V(0,0,lift)),p[n]) for n in ("fixed","cap","rotor","pin")))
    for lift in (8.,5.,2.,1.,0.):
        s=turn(p["cap"],-TURN).translate(V(0,0,lift))
        record(f"cap insertion {lift}",overlap(s,p["fixed"])+overlap(s,p["rotor"]))
    for angle in (-30,-25,-20,-15,-10,-5,0):
        s=turn(p["cap"],angle)
        record(f"cap turn {angle}",overlap(s,p["fixed"])+overlap(s,p["rotor"]))
    for dx in (0.,1.,2.,3.,4.,STROKE):
        s=p["pin"].translate(V(dx,0,0))
        record(f"pin travel {dx}",overlap(s,p["fixed"])+overlap(s,p["rotor"]))
    record("rotor axial retention",overlap(p["rotor"].translate(V(0,0,1.)),p["cap"]),True)
    record("bayonet axial retention",overlap(p["cap"].translate(V(0,0,1.)),p["fixed"]),True)
    record("bayonet end stop",overlap(turn(p["cap"],5),p["fixed"]),True)
    record("unlocked cap extraction",overlap(turn(p["cap"],-TURN).translate(V(0,0,5.)),p["fixed"]))
    record("angular stop",overlap(turn(p["rotor"],15),p["pin"]),True)
    record("pin outward stop",overlap(p["pin"].translate(V(7,0,0)),p["fixed"]),True)
    record("pin inward stop",overlap(p["pin"].translate(V(-2,0,0)),p["rotor"]),True)
    record("pin safety key",overlap(p["pin"].translate(V(1.5,0,0)),p["pin_key"]),True)
    return {"status":"UNLOADED_REVIEW_ONLY","passed":all(r["status"]=="PASS" for r in rows),"checks":rows,
            "continuous_assembly_path":"NOT_RUN","cap_reverse_retention":"NOT_RUN_NO_POSITIVE_STOP","pin_key_retention_under_vibration":"NOT_RUN","full_clamp_joint_integration":"NOT_RUN","load":"NOT_RUN"}
def export(folder):
    folder=Path(folder);folder.mkdir(parents=True,exist_ok=True)
    for ext in ("stl","3mf","step"):
        for name in ("fixed","rotor","cap","cap_key","pin","pin_key"):
            (folder/(name+"."+ext)).unlink(missing_ok=True)
    (folder/"REVIEW_CAP_SECURED_NO_IMPRIMIR.3mf").unlink(missing_ok=True)
    p=parts()
    for name,locked in (("LOCKED",True),("RELEASED",False)):
        q=parts(0 if locked else 15,locked)
        assembly=cq.Assembly()
        for n,s in q.items(): assembly.add(s,name=n)
        assembly.save(str(folder/("REVIEW_"+name+".step")))
        cq.exporters.export(cq.Compound.makeCompound(list(q.values())),str(folder/("REVIEW_"+name+"_NO_IMPRIMIR.3mf")))
    # Explicit review of the three closure stages; radial indexing unchanged.
    for label,angle,lift in (("CAP_ENTRY",-TURN,5.),("CAP_INSERTED",-TURN,0.),("CAP_TURNED",0.,0.)):
        q=parts()
        q["cap"]=turn(q["cap"],angle).translate(V(0,0,lift))
        cq.exporters.export(cq.Compound.makeCompound(list(q.values())),str(folder/("REVIEW_"+label+"_NO_IMPRIMIR.3mf")))
    report=audit()
    (folder/"report.json").write_text(json.dumps(report,indent=2),encoding="utf-8")
    # Bayonet axial retention only; reverse rotation is not positively blocked. Review-only.
    return report
