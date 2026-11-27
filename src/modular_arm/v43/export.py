"""Print-fit kit and bounded diagnostic for V4.3."""
import json
from pathlib import Path
import cadquery as cq
from modular_arm.v43 import joint as j
from modular_arm.v4.chain_motion_screen import save

def export(destination):
    dest=Path(destination); dest.mkdir(parents=True,exist_ok=True)
    p=j.templates()
    report={"status":"unloaded_fit_prototype_only","complete":False,"poses":[],"assembly_path":[],"retention":[]}
    path=dest/"report.json"
    save(path,report)
    for angle,travel in ((0.,0.),(45.,0.),(90.,0.),(180.,0.),(15.,7.),(90.,7.),(165.,7.),(180.,7.)):
        defects=j.audit(j.parts(angle,travel,source=p))
        report["poses"].append({"angle_deg":angle,"travel_mm":travel,"collisions":defects})
        save(path,report)
        print("pose",angle,travel,defects,flush=True)
    report["assembly_path"]=j.insertion_samples(p)
    report["retention"]=j.retention_samples(p)
    report["complete"]=True
    report["clear_samples"]=not any(row["collisions"] for row in report["poses"]) and all(row["overlap_mm3"]<=j.TOL for row in report["assembly_path"])
    save(path,report)
    if not report["clear_samples"]:
        raise ValueError("V4.3 diagnostic failed; inspect report.json before printing")
    if not any(x["lip_keeper_overlap_mm3"]>j.TOL and x["keeper_journal_overlap_mm3"]>j.TOL for x in report["retention"]):
        raise ValueError("axial obstruction not detected")
    printable=dest/"PRINT_STL"; printable.mkdir(exist_ok=True)
    items={"head_a":(p["head_a"],1),"head_b":(p["head_b"],1),
           "cartridge":(p["cartridge"].rotate((0,0,0),(1,0,0),180),1),
           "button":(p["button"],1),"keeper":(p["keeper_0"],4)}
    for name,(shape,qty) in items.items():
        shape=shape.translate(cq.Vector(0,0,-shape.BoundingBox().zmin))
        cq.exporters.export(shape,str(printable/(name+".stl")),tolerance=.04,angularTolerance=.1)
        cq.exporters.export(shape,str(printable/(name+".step")))
    fixture=dest/"MAGNET_TEST"; fixture.mkdir(exist_ok=True)
    cq.exporters.export(j.magnet_coupon(),str(fixture/"magnet_coupon.stl"))
    for angle in (0.,90.,180.):
        preview=dest/f"PREVIEW_{int(angle)}"; preview.mkdir(exist_ok=True)
        for name,shape in j.parts(angle,0.,source=p).items():
            cq.exporters.export(shape,str(preview/(name+".stl")),tolerance=.04,angularTolerance=.1)
    manifest={"status":"unloaded_fit_prototype_only","axis_spacing_mm":29.4,"head_diameter_mm":68.,
              "printable":[{"file":n+".stl","quantity":q} for n,(_,q) in items.items()],
              "hardware":"4 axial M3 x 8 screws and compatible heat-set inserts (nominal; check actual geometry), existing spring candidate, existing D+M4 arm interface.",
              "assembly":"Lower head, upper shell over journal, four keeper sectors inserted from top and slid inward, button with spring, top cartridge, four axial screws. Do not use without cap secured. Magnet coupon separate; not installed.",
              "limitations":"Discrete CAD checks, no force, fatigue, fastener seating, heat-set pullout, continuous motion or full-chain validation. Larger diameter is deliberate; original V4.1 unchanged."}
    (dest/"manifest.json").write_text(json.dumps(manifest,indent=2),encoding="utf-8")
    return manifest
