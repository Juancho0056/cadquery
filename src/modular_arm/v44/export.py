"""Generate complete compact joint only after discrete CAD checks."""
import json
from pathlib import Path
import cadquery as cq
from modular_arm.v44 import joint as j
from modular_arm.v4.chain_motion_screen import save

def export(destination):
    dest=Path(destination); dest.mkdir(parents=True,exist_ok=True)
    p=j.templates()
    result={"status":"compact_fit_candidate","diameter_mm":56.,"axis_spacing_mm":29.4,"poses":[],"path":[],"complete":False}
    path=dest/"report.json"; save(path,result)
    for angle,travel in ((0.,0.),(90.,0.),(180.,0.),(15.,7.),(165.,7.)):
        defects=j.audit(j.parts(angle,travel,p))
        result["poses"].append({"angle":angle,"travel":travel,"collisions":defects})
        save(path,result); print("pose",angle,travel,defects,flush=True)
    result["path"]=j.path_samples(p)
    result["retention"]=j.retention(p)
    result["complete"]=True
    result["passed"]=not any(x["collisions"] for x in result["poses"]) and all(x["overlap_mm3"]<=j.TOL for x in result["path"]) and min(result["retention"].values())>j.TOL
    save(path,result)
    if not result["passed"]: raise ValueError("Compact diagnostic rejected; inspect report.json")
    folder=dest/"PRINT_STL"; folder.mkdir(exist_ok=True)
    items={n:(p[n],1) for n in ("head_a","head_b","cap","key","cartridge","button")}
    items["keeper"]=(p["keeper_0"],4)
    for name,(shape,count) in items.items():
        if name in ("cap","cartridge"): shape=shape.rotate((0,0,0),(1,0,0),180)
        shape=shape.translate(j.V(0,0,-shape.BoundingBox().zmin))
        cq.exporters.export(shape,str(folder/(name+".stl")),tolerance=.04,angularTolerance=.1)
        cq.exporters.export(shape,str(folder/(name+".step")))
    for angle in (0.,180.):
        preview=dest/f"PREVIEW_{int(angle)}"; preview.mkdir(exist_ok=True)
        for name,shape in j.parts(angle,0.,p).items(): cq.exporters.export(shape,str(preview/(name+".stl")))
    manifest={"status":"unloaded_fit_prototype_only","diameter_mm":56,"axis_spacing_mm":29.4,
              "printable":{n:q for n,(_,q) in items.items()},
              "assembly":"Head A, head B, four keeper sectors, spring/button, keyed cartridge, threaded cap, removable seam key.",
              "limitations":"No radial or cap screws; D+M4 retained. Key is not captive; spring only, magnets not qualified. No continuous clearance, load rating or full chain validation."}
    (dest/"manifest.json").write_text(json.dumps(manifest,indent=2),encoding="utf-8")
    return manifest
