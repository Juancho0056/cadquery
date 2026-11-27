"""Original A/B telescopic pairing hypothesis, using actual full distal bodies.
The two modular tubes stay unchanged. No long clamp screw is involved.
All translations in mm, neutral V4.1 frame.
"""
import math,json
from pathlib import Path
import numpy as np
from modular_arm.v4 import integrated_arm as ia,modular_connection as m
from modular_arm.v4.clocking import matrix

MOUTH_Z=145.67807006835938
DEPTHS=(20.,35.,50.,65.,80.,95.)
CLOCKINGS=(0.,90.,180.,270.)

def counterpart_transform(side,depth,clock=0.):
    if side not in ("a","b") or not math.isfinite(depth) or not 0<=depth<=100:
        raise ValueError("invalid extension candidate")
    # Source +Z maps to +X for BOTH bodies. Original inner tip is Z=0,
    # outer mouth is Z=MOUTH_Z. Preserve those two datums.
    index=5 if side=="a" else 1
    fixed_index=1 if side=="a" else 5
    cx,cy=ia.CENTERS[index]
    origin=np.eye(4)
    origin[:3,3]=(-cx,-cy,-(MOUTH_Z if side=="a" else 0.))
    rot=np.array([[0,0,1,0],[0,1,0,0],[-1,0,0,0],[0,0,0,1]],float)
    x=(-ia.JOIN_X-ia.CUTS[1]+depth if side=="a"
       else ia.JOIN_X+MOUTH_Z-ia.CUTS[5]-depth)
    place=np.eye(4)
    place[:3,3]=(x,0,m.flush_arm.AXES[fixed_index])
    return place@matrix((180.+clock)%360)@rot@origin

def scan(archive,destination,depths=DEPTHS,clocks=CLOCKINGS):
    fixed=m.parts(archive,0.,0.)
    originals={s:ia.original_mesh(archive,i) for s,i in (("a",5),("b",1))}
    rows=[]
    accepted={}
    for side in ("a","b"):
        candidates=[]
        for depth in depths:
            for clock in clocks:
                moving=originals[side].copy()
                moving.apply_transform(counterpart_transform(side,depth,clock))
                try:
                    collisions=ia.audit({**fixed,"distal_candidate":moving})
                    row={"side":side,"depth_mm":depth,"relative_clock_deg":clock,
                         "collisions":collisions,"clear":not collisions}
                except (ValueError,RuntimeError) as exc:
                    row={"side":side,"depth_mm":depth,"relative_clock_deg":clock,
                         "error":str(exc),"clear":False}
                rows.append(row)
                print(json.dumps(row),flush=True)
                if row["clear"]:
                    candidates.append((row,moving))
        accepted[side]=candidates
    # Prefer deepest paired clear poses, but check BOTH distal parts together.
    selected=None
    for ra,a in sorted(accepted["a"],key=lambda x:-x[0]["depth_mm"]):
        for rb,b in sorted(accepted["b"],key=lambda x:-x[0]["depth_mm"]):
            combined={**fixed,"distal_outer_a":a,"distal_inner_b":b}
            try:
                if ia.audit(combined):
                    continue
            except (ValueError,RuntimeError):
                continue
            selected=(ra,rb,combined)
            break
        if selected:
            break
    dest=Path(destination)
    dest.mkdir(parents=True,exist_ok=True)
    preview=dest/"candidate_preview"
    preview.mkdir(exist_ok=True)
    for p in preview.glob("*.stl"):
        p.unlink()
    if selected:
        for name,body in selected[2].items():
            body.export(preview/(name+".stl"))
    result={"status":"original_pair_discrete_candidate_only","rows":rows,
            "selected_pair":list(selected[:2]) if selected else None,
            "limitations":"Assumes obj1 inner and obj5 outer. Original distal forks retained. No transverse locking screw, thread alignment, continuous travel, folded pose or load rating verified."}
    (dest/"report.json").write_text(json.dumps(result,indent=2),encoding="utf-8")
    return result
