"""45 mm circular housing, two split keepers, internal bayonet cap.
All dimensions mm. Independent unloaded fit candidate.
"""
import math
from itertools import combinations
import cadquery as cq
from modular_arm.v43 import joint as old
from modular_arm.v43.closure_coupons import turn
from modular_arm.v3.locking_core import Core,cyl,profile_points
from modular_arm.v4.transition import rounded_wire
V=cq.Vector
R=22.5
JOURNAL=16.5
GROOVE=15.
KEEPER_IN=15.25
KEEPER_OUT=17.6
POCKET=19.8
SCALE=.72
STEM=7.92
LUG_ROOT=19.45
LUG_TIP=21.05
ENTRY_ANGLES=(45.,165.,285.)
LOCK_ANGLE=30.
LUG_HALF=8.
TRACK_HALF=9.
LUG_Z=36.1
LUG_HEIGHT=2.4
TRACK_Z=35.8
TRACK_HEIGHT=3.0
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
    b=b.cut(cyl(9.,-7.,7.1)).cut(teeth(-1.35,20.,.35))
    # Integral spring floor at z=-7; no ring intruding into plunger stroke.
    return old.socket(b,1)

def guide(a,z,h,clear=0.):
    return turn(cq.Solid.makeBox(2.0+clear,3.+2*clear,h,V(18.9,-1.5-clear,z)),a)

def sector(a,lo,hi,z,h,sweep):
    b=cq.Solid.makeCylinder(hi,h,V(0,0,z),V(0,0,1),sweep)
    return turn(b.cut(cyl(lo,z-1.,h+2.)),a)

def bayonet_tracks(b):
    for a in ENTRY_ANGLES:
        # Vertical entrance plus blind circumferential track. No helix.
        b=b.cut(sector(a-TRACK_HALF,19.35,21.35,TRACK_Z,7.,2*TRACK_HALF))
        b=b.cut(sector(a-TRACK_HALF,19.35,21.35,TRACK_Z,TRACK_HEIGHT,LOCK_ANGLE+2*TRACK_HALF))
    return b

def upper():
    b=blank(5).cut(cyl(JOURNAL+.3,12.,2.7)).cut(cyl(POCKET,14.7,27.))
    b=bayonet_tracks(b)
    for a in (90.,270.): b=b.cut(guide(a,20.,22.,.2))
    return old.socket(b,5)

def keeper(i):
    if i not in (0,1): raise ValueError("keeper index")
    # Two opposing 100-degree sectors; rigid insertion corridor within Ø39.6.
    b=cq.Solid.makeCylinder(KEEPER_OUT,2.6,V(0,0,15.1),V(0,0,1),100.)
    return valid(turn(b,40.+180*i).cut(cyl(KEEPER_IN,14.,5.)),"keeper")

def cartridge():
    b=ring(19.45,17.9,14.85,4.15).fuse(cyl(19.45,18.85,15.35))
    for a in (90.,270.): b=b.fuse(guide(a,20.3,8.))
    b=b.cut(teeth(18.85,TOOTH_TOP-18.85,.35))
    b=b.cut(cyl(STEM+.35,TOOTH_TOP-.1,10.))
    return valid(b,"cartridge")

def cap():
    b=ring(LUG_ROOT,STEM+.35,CAP_Z,7.2)
    for a in ENTRY_ANGLES:
        b=b.fuse(sector(a+LOCK_ANGLE-LUG_HALF,LUG_ROOT-.2,LUG_TIP,LUG_Z,LUG_HEIGHT,2*LUG_HALF))
    b=b.fuse(ring(R,STEM+.35,41.45,2.15))
    for a in range(15,360,30):
        t=math.radians(a)
        b=b.cut(cq.Solid.makeCylinder(1.5,2.3,V(22.5*math.cos(t),22.5*math.sin(t),41.4)))
    return valid(b,"cap")

def button():
    return valid(teeth(6.,19.).fuse(cyl(STEM,24.9,26.5)),"button")

def lock_pair(b,c):
    hole=cq.Solid.makeBox(1.2,2.4,4.7,V(21.,-1.2,38.8))
    key=cq.Solid.makeBox(.9,2.,4.4,V(21.15,-1.,39.))
    return valid(b.cut(hole),"upper"),valid(c.cut(hole),"cap"),valid(key,"key")

# Magnetic variant: integral pads, not loose carriers. Same exterior.
MAGNET_RADIUS=3.
MAGNET_HEIGHT=2.7
MAGNET_POCKET_RADIUS=3.1
MAGNET_POCKET_DEPTH=2.8
MAGNET_PCD_RADIUS=4.7
MAGNET_PAD_RADIUS=8.2
MAGNET_BOTTOM_FACE=-5.
MAGNET_TOP_FACE=4.
def magnet_centres():
    return [(MAGNET_PCD_RADIUS*math.cos(math.radians(a)),MAGNET_PCD_RADIUS*math.sin(math.radians(a))) for a in (0.,120.,240.)]

def magnet_pockets(shape,upward):
    # Floor opens upward; plunger opens downward. Recess .1 mm nominal.
    z=MAGNET_BOTTOM_FACE-MAGNET_POCKET_DEPTH if upward else MAGNET_TOP_FACE-.05
    h=MAGNET_POCKET_DEPTH+.05
    for x,y in magnet_centres():
        shape=shape.cut(cq.Solid.makeCylinder(MAGNET_POCKET_RADIUS,h,V(x,y,z)))
    return valid(shape,"magnet pockets")

def magnetic_lower():
    return magnet_pockets(lower().fuse(cyl(MAGNET_PAD_RADIUS,-7.1,2.1)),True)

def magnetic_button():
    return magnet_pockets(button().fuse(cyl(MAGNET_PAD_RADIUS,4.,2.1)),False)

def magnet_solids(travel=0.):
    return {f"magnet_{side}_{i}":cq.Solid.makeCylinder(MAGNET_RADIUS,MAGNET_HEIGHT,V(x,y,z))
            for side,z in (("lower",MAGNET_BOTTOM_FACE-MAGNET_POCKET_DEPTH),("upper",MAGNET_TOP_FACE+.1-travel))
            for i,(x,y) in enumerate(magnet_centres())}

def magnetic_checks(p):
    rows=[]
    for travel in (0.,1.,2.,3.,4.,5.,6.,7.):
        assembly=parts(0.,travel,p)
        mags=magnet_solids(travel)
        defects=audit({**assembly,**mags})
        rows.append({"travel_mm":travel,"face_gap_mm":9.2-travel,"collisions":defects})
    return rows

def templates(return_mode="spring"):
    if return_mode not in ("spring","magnet"): raise ValueError("return mode")
    b,c,k=lock_pair(upper(),cap())
    return {"head_a":magnetic_lower() if return_mode=="magnet" else lower(),"head_b":b,"cap":c,"key":k,"cartridge":cartridge(),"button":magnetic_button() if return_mode=="magnet" else button(),"keeper_0":keeper(0),"keeper_1":keeper(1)}

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
        for r,z in [(2.1,z) for z in (30.,15.,5.,2.,1.,.5,0.)]+[(r,0.) for r in (1.8,1.5,1.,.5,0.)]:
            sample(n,p[n].translate(V(0,sign*r,z)),installed,radial=r,lift=z)
        installed.append(n)
    for n in ("button","cartridge"):
        for dz in (35.,15.,5.,2.,0.):
            sample(n,p[n].translate(V(0,0,dz)),installed,lift=dz)
        installed.append(n)
    for dz in (12.,9.,6.,4.,3.,2.,1.,.5,0.):
        sample("cap_insert",turn(p["cap"],-LOCK_ANGLE).translate(V(0,0,dz)),installed,lift=dz,angle=-LOCK_ANGLE)
    for a in (-30.,-25.,-20.,-15.,-10.,-5.,0.):
        sample("cap_turn",turn(p["cap"],a),installed,lift=0.,angle=a)
    for dz in (5.,2.,0.):
        sample("key",p["key"].translate(V(0,0,dz)),installed+["cap"],lift=dz)
    return rows

def retention(p):
    return {"lip":overlap(p["head_b"].translate(V(0,0,1.)),p["keeper_0"]),
            "groove":overlap(p["keeper_0"].translate(V(0,0,1.)),p["head_a"]),
            "sleeve":overlap(p["keeper_0"].translate(V(0,2.,0)),p["cartridge"]),
            "cap_pull":overlap(p["cap"].translate(V(0,0,1.)),p["head_b"]),
            "key_minus":overlap(turn(p["cap"],-5.),p["key"]),
            "key_plus":overlap(turn(p["cap"],5.),p["key"]),
            "end_stop":overlap(turn(p["cap"],5.),p["head_b"]),
            "unlocked_pull_clear":overlap(turn(p["cap"],-LOCK_ANGLE).translate(V(0,0,2.)),p["head_b"])}
