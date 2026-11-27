"""Visual review always separated from printable output; fail closed on CAD checks."""
import json
from pathlib import Path
import cadquery as cq
from modular_arm.v45 import joint as j
from modular_arm.v4.chain_motion_screen import save

def export(destination,return_mode='spring'):
    dest=Path(destination); dest.mkdir(parents=True,exist_ok=True)
    folder=dest/"PRINT_STL"
    if folder.exists():
        for old in (*folder.glob("*.stl"),*folder.glob("*.step")): old.unlink()
    (dest/"manifest.json").unlink(missing_ok=True)
    p=j.templates(return_mode)
    result={"status":"compact_45_3_bayonet_candidate","return_mode":return_mode,"diameter_mm":45.,"axis_spacing_mm":29.4,"poses":[],"path":[],"complete":False}
    path=dest/"report.json"; save(path,result)
    # Review even if assembly diagnostics fail. Never label these printable.
    for angle in (0.,180.):
        preview=dest/f"REVIEW_{int(angle)}"; preview.mkdir(exist_ok=True)
        for name,shape in j.parts(angle,0.,p).items():
            cq.exporters.export(shape,str(preview/(name+".stl")),tolerance=.04,angularTolerance=.1)
    for angle,travel in ((0.,0.),(90.,0.),(180.,0.),(15.,7.),(165.,7.)):
        defects=j.audit(j.parts(angle,travel,p))
        result["poses"].append({"angle":angle,"travel":travel,"collisions":defects})
        save(path,result); print("pose",angle,travel,defects,flush=True)
    result["path"]=j.path_samples(p)
    result["retention"]=j.retention(p)
    if return_mode=="magnet": result["magnetic_checks"]=j.magnetic_checks(p)
    result["complete"]=True
    result["width_mm"]={n:p[n].BoundingBox().ylen for n in ("head_a","head_b","cap")}
    result["passed"]=max(result["width_mm"].values())<=45.001 and not any(x["collisions"] for x in result["poses"]) and all(x["overlap_mm3"]<=j.TOL for x in result["path"]) and min(v for k,v in result["retention"].items() if k!="unlocked_pull_clear")>j.TOL and result["retention"]["unlocked_pull_clear"]<=j.TOL
    if return_mode=="magnet":
        result["passed"]=result["passed"] and all(not x["collisions"] and x["face_gap_mm"]>0 for x in result["magnetic_checks"])
    save(path,result)
    if not result["passed"]:
        print("Review only: assembly diagnostics failed. Do not print.",flush=True)
        return result
    folder.mkdir(exist_ok=True)
    items={n:(p[n],1) for n in ("head_a","head_b","cap","key","cartridge","button")}
    items["keeper"]=(p["keeper_0"],2)
    for name,(shape,count) in items.items():
        if name in ("cap","cartridge"): shape=shape.rotate((0,0,0),(1,0,0),180)
        shape=shape.translate(j.V(0,0,-shape.BoundingBox().zmin))
        cq.exporters.export(shape,str(folder/(name+".stl")),tolerance=.04,angularTolerance=.1)
        cq.exporters.export(shape,str(folder/(name+".step")))
    manifest={"status":"unloaded_fit_prototype_only","diameter_mm":45,"axis_spacing_mm":29.4,
              "return_mode":return_mode,"printable":{n:q for n,(_,q) in items.items()},
              "magnets":{"count":6,"diameter_mm":6,"height_mm":2.7,"gap_rest_mm":9.2,"gap_pressed_mm":2.2,"installation":"Bond securely with compatible adhesive; same poles facing; verify force before loading."} if return_mode=="magnet" else None,
              "assembly":"Head A, head B, two opposing keeper sectors, return element/button, keyed cartridge, bayonet cap, removable seam key.",
              "limitations":"No radial or cap screws; D+M4 retained. Key not captive. Spring fit and force unverified; magnets not qualified. No continuous clearance, load rating or full chain validation."}
    (dest/"manifest.json").write_text(json.dumps(manifest,indent=2),encoding="utf-8")
    return manifest
