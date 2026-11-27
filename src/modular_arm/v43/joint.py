"""Top-access segmented keeper with independent axial retention.

Original CAD implementation, no third-party meshes. All dimensions mm.
Requires four axial M3 screws/inserts in the removable cartridge, not
radial pins. Unloaded fit prototype, not load rated.
"""
import math
import cadquery as cq
from modular_arm.v4 import separation_variant as s
from modular_arm.v3.locking_core import Core,cyl,profile
m=s.m
V=cq.Vector
DELTA=1.
RADIUS=34.
JOURNAL=23.
GROOVE_ROOT=21.5
KEEPER_IN=21.75
KEEPER_OUT=25.4
KEEPER_Z=15.1
KEEPER_HEIGHT=2.6
POCKET_RADIUS=28.
SLEEVE_IN=25.75
SLEEVE_OUT=27.7
INSERT_OFFSET=2.4
CAP_BOTTOM=18.85
CAP_FLANGE=41.4
CAP_TOP=45.4
TOOTH_TOP=25.
STROKE=7.
FASTENER_ANGLES=(45,135,225,315)
FASTENER_RADIUS=31.
TOL=.01

def ring(ro,ri,z,h):
    return cyl(ro,z,h).cut(cyl(ri,z-1,h+2))

def valid(shape,name):
    return m.valid(shape.clean(),name)

def socket(shape,index):
    t=V(0,0,s.shift(index,DELTA))
    shape=shape.intersect(m.axial_box(index,-60,m.JOIN))
    shape=shape.cut(m.keyed(index,m.JOIN-m.LENGTH-m.END_GAP,m.JOIN+1,m.CLEARANCE).translate(t))
    for seat in m.bearing_seats(index): shape=shape.cut(seat.translate(t))
    return valid(shape.cut(m.pin_bore(index).translate(t)),"D socket")

def exterior(index):
    lo,hi=s.f.LIMITS[index]
    lo-=DELTA if index==1 else 0.
    hi+=DELTA if index==5 else 0.
    # Larger disc provides a distinct load path for keeper and axial cap screws.
    return valid(s.blank(index,DELTA).fuse(cyl(RADIUS,lo,hi-lo)),"exterior")

def lower():
    c=Core()
    body=exterior(1).fuse(cyl(JOURNAL,11.8,6.7))
    body=body.cut(ring(24.,GROOVE_ROOT,14.9,3.))
    body=body.cut(cyl(19.4,-7.,7.1)).cut(profile(c,0.,18.6,c.clearance))
    body=body.fuse(ring(19.8,10.,-3.,2.))
    return socket(body,1)

def fastener_cylinder(radius,z,h,angle):
    a=math.radians(angle)
    return cq.Solid.makeCylinder(radius,h,V(FASTENER_RADIUS*math.cos(a),FASTENER_RADIUS*math.sin(a),z))

def upper():
    body=exterior(5).cut(cyl(23.35,12.,2.7)).cut(cyl(POCKET_RADIUS,14.7,27.))
    # Nominal insert pilot only; verify actual insert manufacturer dimensions.
    for angle in FASTENER_ANGLES:
        body=body.cut(fastener_cylinder(2.1,32.4,9.1,angle))
    return socket(body,5)

def keeper(index=0):
    if index not in range(4): raise ValueError("keeper index 0..3")
    outer=cq.Solid.makeCylinder(KEEPER_OUT,KEEPER_HEIGHT,V(0,0,KEEPER_Z),V(0,0,1),89.)
    outer=outer.rotate((0,0,0),(0,0,1),.5+90*index)
    return valid(outer.cut(cyl(KEEPER_IN,KEEPER_Z-1,KEEPER_HEIGHT+2)),"keeper")

def cartridge():
    c=Core()
    sleeve=ring(SLEEVE_OUT,SLEEVE_IN,14.85,4.15)
    hub=cyl(SLEEVE_OUT,CAP_BOTTOM,CAP_TOP-CAP_BOTTOM)
    flange=cyl(RADIUS,CAP_FLANGE,CAP_TOP-CAP_FLANGE)
    body=sleeve.fuse(hub).fuse(flange)
    body=body.cut(profile(c,CAP_BOTTOM,TOOTH_TOP-CAP_BOTTOM,c.clearance))
    body=body.cut(cyl(c.stem_radius+c.clearance,TOOTH_TOP-.1,CAP_TOP-TOOTH_TOP+1.1))
    for angle in FASTENER_ANGLES:
        body=body.cut(fastener_cylinder(1.65,CAP_FLANGE-.1,4.2,angle))
        body=body.cut(fastener_cylinder(2.85,43.,2.5,angle))
    return valid(body,"cartridge")

def button(travel=0.):
    s.a.validate_pose(0.,travel)
    c=Core()
    body=profile(c,6.,TOOTH_TOP-6.).fuse(cyl(c.stem_radius,TOOTH_TOP-.1,53.4-TOOTH_TOP+.1))
    return valid(body,"button").translate(V(0,0,-travel))

def templates():
    return {"head_a":lower(),"head_b":upper(),"cartridge":cartridge(),
            "button":button(),**{f"keeper_{i}":keeper(i) for i in range(4)}}

def parts(angle=0.,travel=0.,*,source=None):
    s.a.validate_pose(angle,travel)
    p=dict(templates() if source is None else source)
    for n in ("head_b","cartridge",*(f"keeper_{i}" for i in range(4))):
        p[n]=m.joint.turn(p[n],angle)
    p["button"]=p["button"].translate(V(0,0,-travel))
    return p

def overlap(a,b):
    return a.intersect(b).Volume()

def audit(parts):
    from itertools import combinations
    return [(x,y,v) for x,y in combinations(parts,2)
            if (v:=overlap(parts[x],parts[y]))>TOL]

def insertion_samples(source=None):
    p=templates() if source is None else source
    rows=[]
    # B first, before keepers; validate axial assembly path.
    for dz in (40.,20.,10.,5.,2.,1.,0.):
        v=overlap(p["head_a"],p["head_b"].translate(V(0,0,dz)))
        rows.append({"step":"head_b","lift":dz,"overlap_mm3":v})
    installed=[]
    for i in range(4):
        key=f"keeper_{i}"; k=p[key]
        a=math.radians(45+90*i)
        def moved(radial,z):
            return k.translate(V(radial*math.cos(a),radial*math.sin(a),z))
        for radial,z in [(INSERT_OFFSET,t) for t in (35.,25.,15.,5.,3.5,3.,2.,1.,0.)]+[(t,0.) for t in (2.,1.5,1.,.5,0.)]:
            shape=moved(radial,z)
            v=sum(overlap(shape,p[n]) for n in ("head_a","head_b",*installed))
            rows.append({"step":key,"radial":radial,"lift":z,"overlap_mm3":v})
        installed.append(key)
    # Button inserted while cap absent. Cap then slides over aligned button.
    for dz in (45.,30.,15.,7.,3.,1.,0.):
        v=sum(overlap(p["button"].translate(V(0,0,dz)),p[n]) for n in ("head_a","head_b",*installed))
        rows.append({"step":"button","lift":dz,"overlap_mm3":v})
    for dz in (45.,30.,15.,7.,3.,1.,0.):
        v=sum(overlap(p["cartridge"].translate(V(0,0,dz)),p[n]) for n in ("head_a","head_b","button",*installed))
        rows.append({"step":"cartridge","lift":dz,"overlap_mm3":v})
    return rows

def retention_samples(source=None):
    p=templates() if source is None else source
    rows=[]
    # Axial shoulder and ring constraint, independently of the button.
    for dz in (.1,.4,.6,1.,2.):
        body=p["head_b"].translate(V(0,0,dz))
        k=p["keeper_0"].translate(V(0,0,dz))
        rows.append({"lift_mm":dz,
                     "lip_keeper_overlap_mm3":overlap(body,p["keeper_0"]),
                     "keeper_journal_overlap_mm3":overlap(k,p["head_a"])})
    return rows

def magnet_coupon():
    # Separate test fixture only. Three pockets for user-owned 6 x 2.7 magnets.
    # Not part of the articulation and not a qualified return system.
    body=cyl(9.,0.,3.3)
    for i in range(3):
        a=math.radians(i*120)
        pocket=cq.Solid.makeCylinder(3.1,2.8,V(4.7*math.cos(a),4.7*math.sin(a),.5))
        body=body.cut(pocket)
    return valid(body,"magnet force coupon")
