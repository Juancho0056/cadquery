"""Diagnostic transverse screw placement. No thread fit or lock certification."""
import json,math
from pathlib import Path
import numpy as np
from modular_arm.v4 import extension_pair as e

# Original obj22 has the same inventory signature as obj18.
SCREW_CENTER=(105.79920959472656,96.245361328125)
BOSS_Z=135.545
BOSS_TOP_X=167.030517578125
OFFSETS=(-4.,-2.,0.,2.,4.)
DEPTHS=(35.,65.,95.)

def screw_transform(side,depth,offset,clock=0.):
    if side not in ("a","b") or not all(map(math.isfinite,(depth,offset,clock))):
        raise ValueError("invalid lock candidate")
    # Head at source Z=0, screw points toward negative source X of outer tube.
    cx,cy=SCREW_CENTER
    local=np.array([[0,0,-1,BOSS_TOP_X+offset],[1,0,0,0],[0,-1,0,BOSS_Z],[0,0,0,1]],float)
    local[:3,3]+=np.array([0,e.ia.CENTERS[5][1]-cx,BOSS_Z*0+cy])
    # Map screw XY center to outer Y/Z datum, not its print plate coordinates.
    if side=="a":
        outer=e.counterpart_transform("a",depth,0.)
    else:
        outer=np.eye(4)
        ox,oy=e.ia.CENTERS[5]
        outer[:3,:]=[[0,0,1,e.ia.JOIN_X-e.ia.CUTS[5]],[0,-1,0,oy],[1,0,0,e.m.flush_arm.AXES[5]-ox]]
    tm=e.ia.mesh_library()
    spin=tm.transformations.rotation_matrix(math.radians(clock),(0,0,1),point=(cx,cy,0))
    return outer@local@spin

def scan(archive,destination):
    ia=e.ia
    tm=ia.mesh_library()
    fixed=e.m.parts(archive,0.,0.)
    source=ia.original_mesh(archive,22)
    originals={s:ia.original_mesh(archive,i) for s,i in (("a",5),("b",1))}
    rows=[]
    for side in ("a","b"):
        for depth in DEPTHS:
            other=originals[side].copy()
            other.apply_transform(e.counterpart_transform(side,depth,0.))
            outer,inner=(other,fixed["tube_a"]) if side=="a" else (fixed["tube_b"],other)
            for offset in OFFSETS:
                screw=source.copy()
                screw.apply_transform(screw_transform(side,depth,offset))
                row={"side":side,"depth_mm":depth,"head_offset_mm":offset,"clock_deg":0.}
                try:
                    for name,body in (("outer",outer),("inner",inner)):
                        row[name+"_overlap_mm3"]=ia.intersection_volume(tm.boolean.intersection([body,screw],engine="manifold"))
                except (ValueError,RuntimeError) as exc:
                    row["error"]=str(exc)
                rows.append(row)
                print(json.dumps(row),flush=True)
    dest=Path(destination)
    dest.mkdir(parents=True,exist_ok=True)
    result={"status":"transverse_screw_diagnostic_only","rows":rows,
            "assumptions":{"source":22,"boss_source_z_mm":BOSS_Z,"boss_source_x_mm":BOSS_TOP_X,"clock_deg":0.},
            "limitations":"Estimated boss datum; overlap in modeled threads is reported, never ignored. No lock, thread engagement, safe stroke or retention certified."}
    (dest/"report.json").write_text(json.dumps(result,indent=2),encoding="utf-8")
    return result
