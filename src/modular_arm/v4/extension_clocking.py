"""Independent A clocking variant. Keep canonical heads and keyed tenon fixed."""
import json
from pathlib import Path
import numpy as np
from modular_arm.v4 import extension_lock as l
from modular_arm.v4.clocking import matrix
e=l.e
m=e.m
ia=e.ia
LOCK_OFFSET=14.75
LOCK_PHASE=60.
CLOCKS=(0.,180.)
INDICES=(0,6,12)

def module_transform(clock):
    z=m.flush_arm.AXES[1]
    t=np.eye(4)
    t[2,3]=z
    u=np.eye(4)
    u[2,3]=-z
    return t@matrix(clock)@u

def tube_a(archive,clock):
    # Reorient original body only; canonical male key remains in head frame.
    tm=ia.mesh_library()
    body=ia.align_body(ia.crop_terminal(ia.original_mesh(archive,1),1),
                       1,axis_z=m.flush_arm.AXES[1],clocking=(180.+clock)%360)
    male=ia.cad_mesh(m.tenon(1))
    if ia.intersection_volume(tm.boolean.intersection([body,male],engine="manifold"))<=1.:
        raise ValueError("insufficient tenon overlap")
    result=ia.require_solid(tm.boolean.union([body,male],engine="manifold"),"clocked tube A")
    preservation=ia.preservation_report(body,result,1)
    if max(preservation.values())>.05:
        raise ValueError(f"body preservation failed: {preservation}")
    return result

def scan(archive,destination):
    tm=ia.mesh_library()
    base=m.parts(archive,0.,0.)
    originals={"a":ia.original_mesh(archive,5),"b":ia.original_mesh(archive,1)}
    source=ia.original_mesh(archive,22)
    dest=Path(destination)
    dest.mkdir(parents=True,exist_ok=True)
    rows=[]
    for clock in CLOCKS:
        neutral=dict(base)
        if clock:
            neutral["tube_a"]=tube_a(archive,clock)
        transform=module_transform(clock)
        # Cache parts by indexed length. Rigidly rotate the entire A pair,
        # including its transverse screw, about the longitudinal axis.
        cache={}
        for side in ("a","b"):
            for idx in INDICES:
                d=l.indexed_depth(idx)
                distal=originals[side].copy()
                distal.apply_transform(e.counterpart_transform(side,d,0.))
                screw=source.copy()
                screw.apply_transform(l.screw_transform(side,d,LOCK_OFFSET,LOCK_PHASE))
                if side=="a":
                    distal.apply_transform(transform)
                    screw.apply_transform(transform)
                cache[side,idx]=(distal,screw)
        for a in INDICES:
            for b in INDICES:
                # Indexed poses only. Preserve canonical joint angular contract.
                for angle in (0.,90.,180.):
                    parts={k:v.copy() for k,v in neutral.items()}
                    for side,idx in (("a",a),("b",b)):
                        distal,screw=cache[side,idx]
                        parts["distal_"+side]=distal.copy()
                        parts["extension_screw_"+side]=screw.copy()
                    # Use canonical m.parts at nonzero angle for the retainers
                    # and button instead of guessing their ownership.
                    if angle:
                        rot=tm.transformations.rotation_matrix(np.radians(angle),(0,0,1))
                        for k in ("head_b","tube_b","distal_b","extension_screw_b",*(n for n in parts if n.startswith("connection_b_"))):
                            parts[k].apply_transform(rot)
                        for n,s in m.joint.screws(angle).items():
                            parts[n]=ia.cad_mesh(s)
                    defects=ia.audit(parts)
                    row={"clock_a_deg":clock,"index_a":a,"index_b":b,"angle_deg":angle,
                         "collisions":defects,"clear":not defects}
                    rows.append(row)
                    print(json.dumps(row),flush=True)
                    if clock==180. and a==b==12 and angle in (0.,180.) and not defects:
                        preview=dest/f"preview_{int(angle)}"
                        preview.mkdir(exist_ok=True)
                        for k,v in parts.items():
                            v.export(preview/(k+".stl"))
    result={"status":"discrete_clocking_variant_only","rows":rows,
            "lock":{"offset_mm":LOCK_OFFSET,"phase_deg":LOCK_PHASE},
            "limitations":"No continuous folding path, helical insertion, structural qualification or load rating. Canonical V4.1 unchanged. Only three indices per side sampled."}
    (dest/"report.json").write_text(json.dumps(result,indent=2),encoding="utf-8")
    return result
