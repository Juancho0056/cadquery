"""Independent symmetric axis-spacing variant; preserve canonical clutch geometry."""
import math
import json
from pathlib import Path
import cadquery as cq
from modular_arm.v4 import alternating_chain as a, extension_clocking as clock
from modular_arm.v4.transition import plan,rounded_wire
from modular_arm.v3.locking_core import Core,cyl
m,ia,c=a.m,a.ia,a.c
f=m.flush_arm
V=cq.Vector
DELTAS=(0.,1.,2.,3.,4.)
INDICES=(0,6,12)

def shift(index,delta):
    if index not in (1,5) or not math.isfinite(delta) or not 0<=delta<=4:
        raise ValueError("invalid spacing variant")
    return (-1 if index==1 else 1)*delta/2

def blank(index,delta):
    dz=shift(index,delta)
    lo,hi=f.LIMITS[index]
    lo+=min(0.,2*dz)
    hi+=max(0.,2*dz)
    z=f.AXES[index]+dz
    slab=cq.Solid.makeBox(100,100,hi-lo,V(-40,-50,lo))
    clip=cq.Solid.makeBox(62,100,100,V(-30,-50,-40))
    shoulder=plan().intersect(slab).intersect(clip)
    tube=cq.Solid.makeCylinder(f.RADIUS,24,V(24,0,z),V(1,0,0))
    wires=[rounded_wire(30.,19.,(hi-lo)/2,z,.5),
           rounded_wire(45.,f.RADIUS,f.RADIUS,z,13.90)]
    ramp=cq.Solid.makeLoft(wires,ruled=True).intersect(plan()).intersect(slab)
    solid=shoulder.fuse(tube).fuse(ramp).clean()
    return m.valid(m.joint.turn(solid,180) if index==1 else solid,"spacing blank")

def head(index,delta):
    if delta==0: return m.head(index)
    dz=shift(index,delta)
    t=V(0,0,dz)
    s=m.joint.fixed(blank(index,delta)) if index==1 else m.joint.moving(blank(index,delta))
    if index==5:
        core=Core()
        s=s.cut(cyl(core.stem_radius+core.clearance,23.3,18.1+delta))
    s=s.intersect(m.axial_box(index,-60,m.JOIN))
    s=s.cut(m.keyed(index,m.JOIN-m.LENGTH-m.END_GAP,m.JOIN+1,m.CLEARANCE).translate(t))
    for seat in m.bearing_seats(index): s=s.cut(seat.translate(t))
    return m.valid(s.cut(m.pin_bore(index).translate(t)).clean(),"spacing head")

def button(delta,travel):
    shift(5,delta)
    a.validate_pose(0.,travel)
    if not delta: return f.button(travel)
    # Outer face grows by delta; extend only the exposed stem.
    core=Core()
    s=f.button(0.).fuse(cyl(core.stem_radius,48.3,delta+.1)).clean()
    return m.valid(s.translate(V(0,0,-travel)),"spacing button")

def build(archive,delta,indices,angle,travel,*,source,cache,heads,buttons):
    # Single joint with universal D ends, not a terminal-fork assembly.
    parts,_=a.assemble(archive,(angle,),indices,source=source,
                       travels=(travel,),joint_meshes=cache)
    inner,outer,screw,flipped=source
    # Incoming segment is outward-clocked; its mating D remains canonical.
    for name in ("segment_0_outer","segment_0_lock"):
        parts[name].apply_transform(clock.module_transform(180.))
    parts["segment_0_inner"]=flipped.copy()
    for name,mesh in parts.items():
        if name.startswith(("segment_0_","joint_0_a_")):
            mesh.apply_translation((0,0,shift(1,delta)))
        elif name.startswith(("segment_1_","joint_0_b_")):
            mesh.apply_translation((0,0,shift(5,delta)))
    parts["joint_0_head_a"]=heads[1].copy()
    parts["joint_0_head_b"]=heads[5].copy()
    parts["joint_0_head_b"].apply_transform(c.rotation(angle))
    parts["joint_0_button"]=buttons[travel].copy()
    return parts

def scan(archive,destination):
    dest=Path(destination); dest.mkdir(parents=True,exist_ok=True)
    source=a.templates(archive)
    cache=a.JointMeshes()
    rows=[]
    selected=None
    def save():
        result={"status":"local_spacing_screen_only","rows":rows,
                "selected_delta_mm":selected,
                "limitations":"Single joint, two reusable D-ended segments, incoming outward-clocked. Discrete intersection tests only; no positive clearance guarantee, global chain compatibility, continuous motion or strength rating."}
        tmp=dest/"report.tmp"
        tmp.write_text(json.dumps(result,indent=2),encoding="utf-8")
        tmp.replace(dest/"report.json")
        return result
    for delta in DELTAS:
        heads={i:ia.cad_mesh(head(i,delta)) for i in (1,5)}
        buttons={t:ia.cad_mesh(button(delta,t)) for t in (0.,7.)}
        passed=True
        # Fail fast on maximum insertion, then check all sampled lengths.
        cases=[((12,12),180.,0.)]
        cases.extend(((i,j),180.,0.) for i in INDICES for j in INDICES if (i,j)!=(12,12))
        cases.extend(((i,i),angle,t) for i in INDICES for angle,t in ((0.,0.),(90.,0.),(135.,0.),(165.,7.),(180.,7.),(-165.,7.)))
        for indices,angle,travel in cases:
            parts=build(archive,delta,indices,angle,travel,source=source,cache=cache,heads=heads,buttons=buttons)
            defects=ia.audit(parts)
            row={"delta_mm":delta,"axis_spacing_mm":28.4+delta,"indices":indices,
                 "angle_deg":angle,"travel_mm":travel,"collisions":defects,"clear":not defects}
            rows.append(row); save()
            print(json.dumps(row),flush=True)
            if defects:
                passed=False
                break
        if passed:
            selected=delta
            for angle in (0.,180.):
                preview=dest/f"preview_{int(angle)}"
                preview.mkdir(exist_ok=True)
                parts=build(archive,delta,(12,12),angle,0.,source=source,cache=cache,heads=heads,buttons=buttons)
                for name,mesh in parts.items(): mesh.export(preview/(name+".stl"))
            break
    return save()
