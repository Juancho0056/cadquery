"""Local diagnostic only. Never exports a printable package."""
import json
from pathlib import Path
import numpy as np
from modular_arm.v4.integrated_arm import parts,mesh_library,intersection_volume,JOIN_X

def report_collision(a,b):
    tm=mesh_library()
    q=tm.boolean.intersection([a,b],engine="manifold")
    volume=intersection_volume(q)
    if volume<=.01:
        return None,q
    lo,hi=q.bounds
    # Folded arms occupy negative X at 180 degrees.
    region=("longitudinal" if hi[0]<-JOIN_X-2 else
            "joint_or_transition" if lo[0]>-JOIN_X+2 else "mixed_or_join")
    return {"volume_mm3":volume,"bounds_mm":[lo.tolist(),hi.tolist()],
            "region":region},q

def export_diagnostic(archive,destination,angle=180.):
    p=parts(archive,angle,0.,flush=True)
    dest=Path(destination)
    dest.mkdir(parents=True,exist_ok=True)
    row,q=report_collision(p["arm_a_integrated"],p["arm_b_integrated"])
    for n,m in p.items():
        m.export(dest/(n+".stl"))
    if row:
        q.export(dest/"COLLISION_NOT_PRINTABLE.stl")
    else:
        (dest/"COLLISION_NOT_PRINTABLE.stl").unlink(missing_ok=True)
    result={"status":"diagnostic_only","angle":angle,"collision":row}
    (dest/"report.json").write_text(json.dumps(result,indent=2),encoding="utf-8")
    return result
