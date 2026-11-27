"""Assembly and axial restraint diagnostic for original transverse screws.
Discrete collision checks, not proof of mechanical locking or thread fit.
"""
import json
from pathlib import Path
import numpy as np
from modular_arm.v4 import extension_lock as l

LOCK_OFFSET=18.
SHIFTS=(-5.,-2.5,-1.,-.5,.5,1.,2.5,5.)
POSES=((0.,0.),(90.,0.),(180.,0.),(30.,7.))

def overlap(a,b):
    ia=l.e.ia
    if np.any(a.bounds[1]<b.bounds[0]) or np.any(b.bounds[1]<a.bounds[0]):
        return 0.
    return ia.intersection_volume(ia.mesh_library().boolean.intersection([a,b],engine="manifold"))

def restraint(inner,screw):
    # Relative translation of inner versus held outer/screw. Both signs.
    rows=[]
    for dx in SHIFTS:
        moved=inner.copy()
        moved.apply_translation((dx,0.,0.))
        rows.append({"shift_mm":dx,"screw_overlap_mm3":overlap(moved,screw)})
    return rows

def report(archive,destination):
    e=l.e
    ia=e.ia
    tm=ia.mesh_library()
    source=ia.original_mesh(archive,22)
    originals={s:ia.original_mesh(archive,i) for s,i in (("a",5),("b",1))}
    fixed=e.m.parts(archive,0.,0.)
    rows=[]
    for side in ("a","b"):
        for idx in range(l.HOLE_COUNT):
            depth=l.indexed_depth(idx)
            other=originals[side].copy()
            other.apply_transform(e.counterpart_transform(side,depth,0.))
            screw=source.copy()
            screw.apply_transform(l.screw_transform(side,depth,LOCK_OFFSET))
            outer,inner=(other,fixed["tube_a"]) if side=="a" else (fixed["tube_b"],other)
            row={"side":side,"index":idx,"depth_mm":depth,
                 "outer_overlap_mm3":overlap(outer,screw),
                 "inner_overlap_mm3":overlap(inner,screw),
                 "tube_overlap_mm3":overlap(outer,inner),
                 "axial_shift_samples":restraint(inner,screw)}
            row["nominal_clear"]=max(row[k] for k in ("outer_overlap_mm3","inner_overlap_mm3","tube_overlap_mm3"))<=.01
            samples=row["axial_shift_samples"]
            row["axial_obstruction_both_directions"]=all(any(r["shift_mm"]*sign>0 and r["screw_overlap_mm3"]>.01 for r in samples) for sign in (-1,1))
            rows.append(row)
            print(json.dumps(row),flush=True)
    poses=[]
    depth=l.indexed_depth(12)
    dest=Path(destination)
    dest.mkdir(parents=True,exist_ok=True)
    preview=dest/"candidate_preview"
    preview.mkdir(exist_ok=True)
    for p in preview.glob("*.stl"):
        p.unlink()
    for angle,travel in POSES:
        parts=e.m.parts(archive,angle,travel)
        for side in ("a","b"):
            other=originals[side].copy()
            other.apply_transform(e.counterpart_transform(side,depth,0.))
            screw=source.copy()
            screw.apply_transform(l.screw_transform(side,depth,LOCK_OFFSET))
            if side=="b":
                rot=tm.transformations.rotation_matrix(np.radians(angle),(0,0,1))
                other.apply_transform(rot)
                screw.apply_transform(rot)
            parts["distal_"+side]=other
            parts["extension_screw_"+side]=screw
        defects=ia.audit(parts)
        poses.append({"angle_deg":angle,"travel_mm":travel,"collisions":defects,"clear":not defects})
        if angle==0. and not defects:
            for name,body in parts.items():
                body.export(preview/(name+".stl"))
    result={"status":"discrete_restraint_diagnostic_not_load_rated",
            "selected_offset_mm":LOCK_OFFSET,"rows":rows,"assembly_poses":poses,
            "limitations":"Fixed screw hypothesis. Obstruction at discrete shifts does not prove positive lock, thread engagement, continuous kinematics or safe stroke. Preview only; no print qualification."}
    (dest/"report.json").write_text(json.dumps(result,indent=2),encoding="utf-8")
    return result
