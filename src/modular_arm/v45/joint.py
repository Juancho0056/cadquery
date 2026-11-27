"""45 mm circular housing, two split keepers, internal screw cap.
All dimensions mm. Independent unloaded fit candidate.
"""
import math
from itertools import combinations
import cadquery as cq
from modular_arm.v43 import joint as old
from modular_arm.v43.closure_coupons import thread,turn
from modular_arm.v3.locking_core import Core,cyl,profile_points
from modular_arm.v4.transition import rounded_wire
V=cq.Vector
R=22.5
JOURNAL=16.5
GROOVE=15.
KEEPER_IN=15.25
KEEPER_OUT=18.
POCKET=19.8
SCALE=.72
STEM=7.92
PITCH=3.
THREAD_ROOT=19.45
THREAD_DEPTH=.85
TOP=41.4
CAP_Z=34.4
TOOTH_TOP=25.
TOL=.01
ring,valid=old.ring,old.valid

def teeth(z,h,clear=0.):
    pts=[(x*SCALE,y*SCALE) for x,y in profile_points(Core(),clear/SCALE)]
    return cq.Workplane("XY").polyline(pts).close().extrude(h).translate((0,0,z)).val()

def blank(index):
    lo,hi=old.s.f.LIMITS[index]
    lo-=1. if index==1 else 0.
    hi+=1. if index==5 else 0.
    z=old.s.f.AXES[index]+old.s.shift(index,1.)
    # New circular envelope, not the original radius-28 shoulder.
    disc=cyl(R,lo,hi-lo)
    neck=cq.Solid.makeLoft([rounded_wire(12.,17.,14.,z,.6),
                            rounded_wire(22.,16.,14.,z,2.),
                            rounded_wire(32.,14.5,14.,z,8.),
                            rounded_wire(45.,13.94,13.94,z,13.90),
                            rounded_wire(48.,13.94,13.94,z,13.90)],ruled=True)
    b=valid(disc.fuse(neck),"compact exterior")
    return turn(b,180.) if index==1 else b

def lower():
    b=blank(1).fuse(cyl(JOURNAL,11.8,6.7))
    b=b.cut(ring(17.,GROOVE,14.9,3.))
    b=b.cut(cyl(9.,-7.,7.1)).cut(teeth(0.,18.6,.35))
    # Plunger stop, permits nominal OD <= 16 mm spring.
    b=b.fuse(ring(11.,8.5,-3.,2.))
    return old.socket(b,1)

def guide(a,z,h,clear=0.):
    return turn(cq.Solid.makeBox(2.0+clear,3.+2*clear,h,V(18.9,-1.5-clear,z)),a)

def upper():
    b=blank(5).cut(cyl(JOURNAL+.3,12.,2.7)).cut(cyl(POCKET,14.7,27.))
    b=b.cut(thread(POCKET,THREAD_DEPTH,2.35,9.).translate(V(0,0,32.2)))
    for a in (90.,270.): b=b.cut(guide(a,20.,22.,.2))
    return old.socket(b,5)

def keeper(i):
    if i not in (0,1): raise ValueError("keeper index")
    # Two nearly semicircular pieces. Insertion path tested separately.
    b=cq.Solid.makeCylinder(KEEPER_OUT,2.6,V(0,0,15.1),V(0,0,1),178.)
    return valid(turn(b,1.+180*i).cut(cyl(KEEPER_IN,14.,5.)),"keeper")

def cartridge():
    b=ring(19.45,18.3,14.85,4.15).fuse(cyl(19.45,18.85,15.35))
    for a in (90.,270.): b=b.fuse(guide(a,20.3,8.))
    b=b.cut(teeth(18.85,TOOTH_TOP-18.85,.35))
    b=b.cut(cyl(STEM+.35,TOOTH_TOP-.1,10.))
    return valid(b,"cartridge")

def cap():
    b=ring(THREAD_ROOT,STEM+.35,CAP_Z,7.)
    b=b.fuse(thread(THREAD_ROOT,THREAD_DEPTH,1.8,5.5).translate(V(0,0,35.2)))
    b=b.fuse(ring(R,STEM+.35,41.35,2.05))
    for a in range(0,360,45):
        t=math.radians(a)
        b=b.cut(cq.Solid.makeCylinder(1.4,.8,V(20.5*math.cos(t),20.5*math.sin(t),42.6)))
    return valid(b,"cap")

def button():
    return valid(teeth(6.,19.).fuse(cyl(STEM,24.9,26.5)),"button")

def lock_pair(b,c):
    hole=cq.Solid.makeBox(1.2,2.4,4.7,V(21.,-1.2,38.8))
    key=cq.Solid.makeBox(.9,2.,4.4,V(21.15,-1.,39.))
    return valid(b.cut(hole),"upper"),valid(c.cut(hole),"cap"),valid(key,"key")

def templates():
    b,c,k=lock_pair(upper(),cap())
    return {"head_a":lower(),"head_b":b,"cap":c,"key":k,"cartridge":cartridge(),"button":button(),"keeper_0":keeper(0),"keeper_1":keeper(1)}

def parts(angle=0.,travel=0.,source=None):
    old.s.a.validate_pose(angle,travel)
    p=dict(templates() if source is None else source)
    for n in p:
        if n not in ("head_a","button"): p[n]=turn(p[n],angle)
    p["button"]=p["button"].translate(V(0,0,-travel))
    return p

def overlap(a,b):
    return a.intersect(b).Volume()

def audit(p):
    return [(x,y,v) for x,y in combinations(p,2) if (v:=overlap(p[x],p[y]))>TOL]

def path_samples(p):
    rows=[]
    def sample(n,shape,others,**pose):
        rows.append({"step":n,**pose,"overlap_mm3":sum(overlap(shape,p[x]) for x in others)})
    for dz in (35.,15.,5.,2.,0.):
        sample("upper",p["head_b"].translate(V(0,0,dz)),["head_a"],lift=dz)
    installed=["head_a","head_b"]
    for i in (0,1):
        n=f"keeper_{i}"; sign=1 if i==0 else -1
        # A rigid half ring may not pass through the narrow annulus.
        # Explicitly reject if the proposed top-access path is blocked.
        for r,z in [(3.,z) for z in (30.,15.,5.,2.,0.)]+[(r,0.) for r in (2.,1.,.5,0.)]:
            sample(n,p[n].translate(V(0,sign*r,z)),installed,radial=r,lift=z)
        installed.append(n)
    for n in ("button","cartridge"):
        for dz in (35.,15.,5.,2.,0.):
            sample(n,p[n].translate(V(0,0,dz)),installed,lift=dz)
        installed.append(n)
    for dz in (12.,9.,6.,3.,2.25,1.5,.75,0.):
        sample("cap",turn(p["cap"],dz/PITCH*360).translate(V(0,0,dz)),installed,lift=dz)
    for dz in (5.,2.,0.):
        sample("key",p["key"].translate(V(0,0,dz)),installed+["cap"],lift=dz)
    return rows

def retention(p):
    return {"lip":overlap(p["head_b"].translate(V(0,0,1.)),p["keeper_0"]),
            "groove":overlap(p["keeper_0"].translate(V(0,0,1.)),p["head_a"]),
            "sleeve":overlap(p["keeper_0"].translate(V(0,2.,0)),p["cartridge"]),
            "cap_pull":overlap(p["cap"].translate(V(0,0,1.)),p["head_b"]),
            "key_minus":overlap(turn(p["cap"],-5.),p["key"]),
            "key_plus":overlap(turn(p["cap"],5.),p["key"])}
