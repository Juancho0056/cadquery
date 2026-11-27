"""Compact internal-thread joint. Fit prototype only; no load rating."""
import math
import cadquery as cq
from modular_arm.v43 import joint as old
from modular_arm.v43.closure_coupons import thread,turn
from modular_arm.v3.locking_core import Core,cyl,profile_points
from itertools import combinations
V=cq.Vector
R=28.
JOURNAL=20.5
GROOVE=19.
KEEPER_IN=19.25
KEEPER_OUT=22.
POCKET=24.5
SCALE=.85
STEM=11*SCALE
PITCH=3.
THREAD_ROOT=24.15
THREAD_DEPTH=1.1
CAP_Z=34.
TOP=41.4
TOOTH_TOP=25.
TOL=.01
ring,valid=old.ring,old.valid

def teeth(z,h,clear=0.):
    pts=[(x*SCALE,y*SCALE) for x,y in profile_points(Core(),clear/SCALE)]
    return cq.Workplane("XY").polyline(pts).close().extrude(h).translate((0,0,z)).val()

def slot(a,z,h,clear=0.):
    return turn(cq.Solid.makeBox(2.+clear,4.+2*clear,h,V(23.5,-2.-clear,z)),a)

def lower():
    b=old.s.blank(1,1.).fuse(cyl(JOURNAL,11.8,6.7))
    b=b.cut(ring(21.,GROOVE,14.9,3.))
    b=b.cut(cyl(19.4,-7.,7.1)).cut(teeth(0.,18.6,.4))
    b=b.fuse(ring(19.8,10.,-3.,2.))
    return old.socket(b,1)

def upper():
    b=old.s.blank(5,1.).cut(cyl(JOURNAL+.35,12.,2.7)).cut(cyl(POCKET,14.7,27.))
    cutter=thread(POCKET,THREAD_DEPTH,2.35,12.).translate(V(0,0,31.8))
    b=b.cut(cutter)
    for a in (90.,270.):
        b=b.cut(slot(a,20.,22.,.2))
    return old.socket(b,5)

def keeper(i):
    if i not in range(4): raise ValueError("keeper index")
    b=cq.Solid.makeCylinder(KEEPER_OUT,2.6,V(0,0,15.1),V(0,0,1),89.)
    return valid(turn(b,.5+90*i).cut(cyl(KEEPER_IN,14.,5.)),"keeper")

def cartridge():
    b=ring(24.15,22.35,14.85,4.15).fuse(cyl(24.15,18.85,14.95))
    for a in (90.,270.): b=b.fuse(slot(a,20.3,8.))
    b=b.cut(teeth(18.85,TOOTH_TOP-18.85,.4))
    b=b.cut(cyl(STEM+.4,TOOTH_TOP-.1,10.))
    return valid(b,"keyed cartridge")

def cap():
    b=ring(THREAD_ROOT,STEM+.4,CAP_Z,7.4)
    b=b.fuse(thread(THREAD_ROOT,THREAD_DEPTH,1.8,5.5).translate(V(0,0,34.8)))
    b=b.fuse(ring(R,STEM+.4,41.2,2.2))
    # Shallow face grip recesses; do not cut into threaded wall.
    for a in range(0,360,45):
        t=math.radians(a)
        b=b.cut(cq.Solid.makeCylinder(2.,1.1,V(25.*math.cos(t),25.*math.sin(t),42.3)))
    return valid(b,"cap")

def button():
    return valid(teeth(6.,19.).fuse(cyl(STEM,24.9,26.5)),"button")

def lock_pair(b,c):
    # Removable top-access seam key; not a captive latch.
    hole=cq.Solid.makeBox(1.6,3.2,4.7,V(26.1,-1.6,38.8))
    key=cq.Solid.makeBox(1.2,2.8,4.4,V(26.3,-1.4,39.))
    return valid(b.cut(hole),"upper"),valid(c.cut(hole),"cap"),valid(key,"key")

def templates():
    b,c,k=lock_pair(upper(),cap())
    return {"head_a":lower(),"head_b":b,"cap":c,"key":k,"cartridge":cartridge(),"button":button(),**{f"keeper_{i}":keeper(i) for i in range(4)}}

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
    def sample(n,shape,others):
        rows.append({"step":n,"overlap_mm3":sum(overlap(shape,p[x]) for x in others)})
    for dz in (35.,15.,5.,2.,0.):
        sample("upper",p["head_b"].translate(V(0,0,dz)),["head_a"])
    installed=["head_a","head_b"]
    for i in range(4):
        n=f"keeper_{i}"; a=math.radians(45+90*i)
        for r,z in [(2.3,z) for z in (30.,15.,5.,2.,0.)]+[(r,0.) for r in (1.5,.75,0.)]:
            sample(n,p[n].translate(V(r*math.cos(a),r*math.sin(a),z)),installed)
        installed.append(n)
    for n in ("button","cartridge"):
        for dz in (35.,15.,5.,2.,0.): sample(n,p[n].translate(V(0,0,dz)),installed)
        installed.append(n)
    for dz in (12.,9.,6.,3.,2.25,1.5,.75,0.):
        sample("cap",turn(p["cap"],dz/PITCH*360).translate(V(0,0,dz)),installed)
    for dz in (5.,2.,0.): sample("key",p["key"].translate(V(0,0,dz)),installed+["cap"])
    return rows

def retention(p):
    return {"lip":overlap(p["head_b"].translate(V(0,0,1.)),p["keeper_0"]),
            "groove":overlap(p["keeper_0"].translate(V(0,0,1.)),p["head_a"]),
            "sleeve":overlap(p["keeper_0"].translate(V(2.,2.,0)),p["cartridge"]),
            "cap_pull":overlap(p["cap"].translate(V(0,0,1.)),p["head_b"]),
            "key_minus":overlap(turn(p["cap"],-5.),p["key"]),
            "key_plus":overlap(turn(p["cap"],5.),p["key"])}
