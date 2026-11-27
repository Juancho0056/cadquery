"""Coaxial insertion hypothesis for obj3 into V4.1 B, not original assembly.
Tip coordinate is global X in the neutral V4.1 frame. Source +Z points inward.
No thread engagement, nut, axial retention or functional stroke assumed.
"""
import math,json
from pathlib import Path
import numpy as np
from modular_arm.v4 import integrated_arm as ia, modular_connection as m
from modular_arm.v4.telescopic_screen import CENTERS

SHAFT_LENGTH=162.
TIPS=(45.,52.,53.,60.,80.,100.,120.,140.,152.)
ANGLES=(0.,90.,180.,270.)

def shaft_transform(tip_x,clock_deg=0.):
    if not all(math.isfinite(x) for x in (tip_x,clock_deg)):
        raise ValueError("nonfinite pose")
    cx,cy=CENTERS[3]
    # Proper rotation: source Z -> -X, source X -> +Z.
    origin=np.eye(4)
    origin[:3,3]=(-cx,-cy,-SHAFT_LENGTH)
    rotation=np.array([[0,0,-1,0],[0,1,0,0],[1,0,0,0],[0,0,0,1]],float)
    tm=ia.mesh_library()
    spin=tm.transformations.rotation_matrix(math.radians(clock_deg),(1,0,0))
    place=tm.transformations.translation_matrix((tip_x,0,m.flush_arm.AXES[5]))
    return place@spin@rotation@origin

def scan(archive,destination,*,tips=TIPS,angles=ANGLES):
    tm=ia.mesh_library()
    shaft=ia.original_mesh(archive,3)
    fixed=m.parts(archive,0.,0.)
    dest=Path(destination)
    dest.mkdir(parents=True,exist_ok=True)
    rows=[]
    best=None
    best_mesh=None
    # Reuse each expensive original mesh. Test actual solid overlaps, not a cylinder.
    for tip in tips:
        for angle in angles:
            moving=shaft.copy()
            moving.apply_transform(shaft_transform(tip,angle))
            overlaps={}
            errors={}
            for name,body in fixed.items():
                if np.any(moving.bounds[1]<body.bounds[0]) or np.any(body.bounds[1]<moving.bounds[0]):
                    continue
                try:
                    q=tm.boolean.intersection([body,moving],engine="manifold")
                    overlaps[name]=ia.intersection_volume(q)
                except (ValueError,RuntimeError) as exc:
                    errors[name]=str(exc)
            clear=not errors and all(v<=.01 for v in overlaps.values())
            row={"tip_x_mm":tip,"clock_deg":angle,"overlap_mm3":overlaps,
                 "errors":errors,"sample_clear":clear}
            rows.append(row)
            print(json.dumps(row),flush=True)
            if clear and (best is None or tip<best["tip_x_mm"]):
                best,best_mesh=row,moving
    preview=dest/"candidate_preview"
    preview.mkdir(exist_ok=True)
    # Do not leave a stale shaft preview if no pose succeeds.
    (preview/"shaft_obj3.stl").unlink(missing_ok=True)
    if best_mesh is not None:
        for name,body in fixed.items():
            body.export(preview/(name+".stl"))
        best_mesh.export(preview/"shaft_obj3.stl")
    result={"status":"discrete_insertion_hypothesis_not_functional_assembly",
            "rows":rows,"deepest_sample_clear":best,
            "limits":"Neutral joint only. Tip-first insertion from distal end, assumed coaxial. No actual thread engagement, continuous path, retention, functional stroke or load rating verified."}
    (dest/"report.json").write_text(json.dumps(result,indent=2),encoding="utf-8")
    return result
