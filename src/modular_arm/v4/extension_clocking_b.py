"""Evaluate B radial orientation with A outward; no head changes."""
import json
from pathlib import Path
import numpy as np
from modular_arm.v4 import extension_clocking as c
from modular_arm.v4.clocking import matrix
m,ia,e,l=c.m,c.ia,c.e,c.l

def transform_b(clock):
    z=m.flush_arm.AXES[5]
    t=np.eye(4); t[2,3]=z
    u=np.eye(4); u[2,3]=-z
    return t@matrix(clock)@u

def tube_b(archive,clock):
    tm=ia.mesh_library()
    body=ia.align_body(ia.crop_terminal(ia.original_mesh(archive,5),5),5,
                       axis_z=m.flush_arm.AXES[5],clocking=(180.+clock)%360)
    male=ia.cad_mesh(m.tenon(5))
    if ia.intersection_volume(tm.boolean.intersection([body,male],engine="manifold"))<=1:
        raise ValueError("insufficient tenon overlap")
    result=ia.require_solid(tm.boolean.union([body,male],engine="manifold"),"clocked B")
    if max(ia.preservation_report(body,result,5).values())>.05:
        raise ValueError("body preservation failed")
    return result

def scan(archive,destination):
    tm=ia.mesh_library()
    base=m.parts(archive,0.,0.)
    base["tube_a"]=c.tube_a(archive,180.)
    originals={"a":ia.original_mesh(archive,5),"b":ia.original_mesh(archive,1)}
    source=ia.original_mesh(archive,22)
    dest=Path(destination); dest.mkdir(parents=True,exist_ok=True)
    rows=[]
    for clock in (90.,270.):
        neutral=dict(base); neutral["tube_b"]=tube_b(archive,clock)
        cache={}
        for side in ("a","b"):
            for idx in c.INDICES:
                depth=l.indexed_depth(idx)
                body=originals[side].copy()
                body.apply_transform(e.counterpart_transform(side,depth,0.))
                screw=source.copy()
                screw.apply_transform(l.screw_transform(side,depth,c.LOCK_OFFSET,c.LOCK_PHASE))
                t=c.module_transform(180.) if side=="a" else transform_b(clock)
                for mesh in (body,screw): mesh.apply_transform(t)
                cache[side,idx]=(body,screw)
        for idxa in c.INDICES:
            for idxb in c.INDICES:
                for angle in (0.,90.,180.):
                    parts={k:v.copy() for k,v in neutral.items()}
                    for side,idx in (("a",idxa),("b",idxb)):
                        body,screw=cache[side,idx]
                        parts["distal_"+side]=body.copy()
                        parts["extension_screw_"+side]=screw.copy()
                    if angle:
                        rot=tm.transformations.rotation_matrix(np.radians(angle),(0,0,1))
                        for k in ("head_b","tube_b","distal_b","extension_screw_b",*(n for n in parts if n.startswith("connection_b_"))):
                            parts[k].apply_transform(rot)
                        for n,s in m.joint.screws(angle).items():
                            parts[n]=ia.cad_mesh(s)
                    defects=ia.audit(parts)
                    row={"clock_b_deg":clock,"index_a":idxa,"index_b":idxb,"angle_deg":angle,"collisions":defects,"clear":not defects}
                    rows.append(row)
                    print(json.dumps(row),flush=True)
                    if idxa==idxb==12 and not defects and angle in (0.,180.):
                        preview=dest/f"preview_b{int(clock)}_{int(angle)}"
                        preview.mkdir(exist_ok=True)
                        for k,v in parts.items(): v.export(preview/(k+".stl"))
    result={"status":"discrete_b_clocking_screen","rows":rows,
            "limitations":"A clock180 fixed; B clock90/270 hypotheses. No continuous motion or load rating. No head modifications."}
    (dest/"report.json").write_text(json.dumps(result,indent=2),encoding="utf-8")
    return result
