"""Independent closure fit coupons; not structural joint parts.
Both use a removable axial key to prevent accidental opening.
"""
import math,json
from pathlib import Path
import cadquery as cq
from modular_arm.v3.locking_core import cyl
from modular_arm.v43.joint import ring,valid
V=cq.Vector
ROOT=26.
CREST=27.4
PITCH=3.
CLEAR=.35
HEIGHT=12.
KEY_W=3.
KEY_R=30.
ANGLES=(0.,120.,240.)

def turn(s,a):
    return s.rotate((0,0,0),(0,0,1),a)

def thread(radius,depth,width,height):
    # Sweep a trapezoidal ridge along a right-handed helix.
    path=cq.Wire.makeHelix(PITCH,height,radius)
    wire=cq.Workplane("XZ").polyline([(radius-.15,-width/2),(radius+depth,-width/4),(radius+depth,width/4),(radius-.15,width/2)]).close()
    return wire.sweep(path,isFrenet=True).val()

def key_slot(angle,z,h,clear=0.):
    return turn(cq.Solid.makeBox(4.+2*clear,KEY_W+2*clear,h,V(KEY_R-2-clear,-KEY_W/2-clear,z)),angle)

def key():
    return valid(cq.Solid.makeBox(4.,KEY_W,5.,V(KEY_R-2,-KEY_W/2,0)).fuse(cq.Solid.makeBox(6.,5.,2.,V(KEY_R-3,-2.5,5.))),"key")

def threaded():
    ridge=thread(ROOT,CREST-ROOT,1.8,9.)
    base=ring(34.,21.,0.,3.).fuse(ring(ROOT,21.,2.8,12.2))
    base=base.fuse(ridge.translate(V(0,0,4.5)))
    # Outer collar closes against the broad base face, not against the clutch.
    cap=ring(34.,ROOT+CLEAR,3.,12.5).fuse(ring(34.,21.,15.3,2.2))
    cutter=thread(ROOT+CLEAR,CREST-ROOT,2.35,12.).translate(V(0,0,4.5-PITCH))
    cap=cap.cut(cutter)
    # Accessible external antirotation key at the seam, installed radially.
    return valid(base,"thread base"),valid(cap,"thread cap")

def wedge(angle,lo,hi,z,h,sweep):
    a=cq.Solid.makeCylinder(hi,h,V(0,0,z),V(0,0,1),sweep)
    return turn(a.cut(cyl(lo,z-1,h+2)),angle)

def bayonet():
    base=ring(34.,21.,0.,3.).fuse(ring(ROOT,21.,2.8,12.2))
    for angle in ANGLES:
        base=base.fuse(wedge(angle,25.8,29.,8.,2.5,22.))
    cap=ring(34.,26.4,3.,12.5).fuse(ring(34.,21.,15.3,2.2))
    for angle in ANGLES:
        # Open axial entrance, followed by a constant-height circumferential track.
        cap=cap.cut(wedge(angle-1,26.2,29.4,2.9,8.,24.))
        for k in range(8):
            cap=cap.cut(wedge(angle-1-5*(k+1),26.2,29.4,7.0,4.0,30.))
    return valid(base,"bayonet base"),valid(cap,"bayonet cap")

def lock_pair(base,cap):
    # Separate radial key bridges the seam in the fully seated position.
    # A broad outer boss on each half provides material for the slot.
    # Key is a fit-test part, not a captive or rated safety device.
    # Slot is horizontal and open outward; key inserts along X.
    slot=cq.Solid.makeBox(6.,3.4,5.4,V(28.,-1.7,.3))
    base=base.cut(slot)
    cap=cap.cut(slot)
    pin=cq.Solid.makeBox(5.8,3.,5.,V(28.2,-1.5,.5))
    return valid(base,"keyed base"),valid(cap,"keyed cap"),valid(pin,"seam key")

def path_samples(name,base,cap):
    if name=="thread":
        poses=[(z/PITCH*360,z) for z in (12.,9.,6.,3.,2.25,1.5,.75,0.)]
    else:
        poses=[(0.,z) for z in (12.,8.,5.,2.,0.)]+[(a,0.) for a in range(5,41,5)]
    return [{"angle_deg":a,"lift_mm":z,"overlap_mm3":base.intersect(turn(cap,a).translate(V(0,0,z))).Volume()} for a,z in poses]

def export(destination):
    dest=Path(destination); dest.mkdir(parents=True,exist_ok=True)
    report={"status":"closure_fit_coupons_only","variants":{}}
    for name,fn in (("thread",threaded),("bayonet",bayonet)):
        base,cap=fn()
        samples=path_samples(name,base,cap)
        # Bayonet final position after rotating cap +40 degrees.
        if name=="bayonet": cap=turn(cap,40.)
        base,cap,pin=lock_pair(base,cap)
        folder=dest/name; folder.mkdir(exist_ok=True)
        for old in (*folder.glob("*.stl"),*folder.glob("*.step")): old.unlink()
        overlap=base.intersect(cap).Volume()
        key_overlap=pin.intersect(base).Volume()+pin.intersect(cap).Volume()
        obstruction=[pin.intersect(turn(cap,a)).Volume() for a in (-5.,5.)]
        pull=base.intersect(cap.translate(V(0,0,2.))).Volume()
        passed=max(overlap,key_overlap,*(x["overlap_mm3"] for x in samples))<=.01 and min(obstruction)> .01 and pull>.01
        report["variants"][name]={"seated_overlap_mm3":overlap,"key_overlap_mm3":key_overlap,
            "path_samples":samples,"key_rotation_obstruction_mm3":obstruction,"axial_obstruction_at_2mm_mm3":pull,
            "status":"candidate" if passed else "diagnostic_failed"}
        if not passed:
            continue
        for label,shape in (("base",base),("cap",cap.rotate((0,0,0),(1,0,0),180)),("key",pin)):
            shape=shape.translate(V(0,0,-shape.BoundingBox().zmin))
            cq.exporters.export(shape,str(folder/(label+".stl")),tolerance=.04,angularTolerance=.1)
            cq.exporters.export(shape,str(folder/(label+".step")))
    (dest/"report.json").write_text(json.dumps(report,indent=2),encoding="utf-8")
    return report
