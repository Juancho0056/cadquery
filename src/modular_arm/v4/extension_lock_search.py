"""Search screw phase and depth, then screen folded configurations independently."""
import json
from pathlib import Path
import numpy as np
from modular_arm.v4 import extension_lock as l, extension_lock_assembly as a

OFFSETS=tuple(14.5+.25*i for i in range(13))
PHASES=tuple(range(0,360,30))
LIMIT=.01
INDICES=range(13)

def search(archive,destination):
    e=l.e
    ia=e.ia
    tm=ia.mesh_library()
    fixed=e.m.parts(archive,0.,0.)
    source=ia.original_mesh(archive,22)
    original_inner=ia.original_mesh(archive,1)
    original_outer=ia.original_mesh(archive,5)
    depth=l.indexed_depth(6)
    inner=original_inner.copy()
    inner.apply_transform(e.counterpart_transform("b",depth,0.))
    outer=fixed["tube_b"]
    rows=[]
    candidates=[]
    # Coarse phase search is not a helical path. No pitch or handedness assumed.
    for offset in OFFSETS:
        for phase in PHASES:
            screw=source.copy()
            screw.apply_transform(l.screw_transform("b",depth,offset,phase))
            row={"offset_mm":offset,"phase_deg":phase}
            row["outer_overlap_mm3"]=a.overlap(outer,screw)
            row["inner_overlap_mm3"]=a.overlap(inner,screw)
            if max(row["outer_overlap_mm3"],row["inner_overlap_mm3"])<=LIMIT:
                samples=a.restraint(inner,screw)
                row["shifts"]=samples
                row["obstruction"]=all(any(r["shift_mm"]*sign>0 and abs(r["shift_mm"])<=2.5 and r["screw_overlap_mm3"]>LIMIT for r in samples) for sign in (-1,1))
                if row["obstruction"]:
                    candidates.append(row)
            rows.append(row)
        print("offset",offset,"candidates",len(candidates),flush=True)
    accepted=None
    verification=[]
    for candidate in sorted(candidates,key=lambda r:r["offset_mm"]):
        checks=[]
        for side in ("a","b"):
            for idx in INDICES:
                d=l.indexed_depth(idx)
                other=(original_outer if side=="a" else original_inner).copy()
                other.apply_transform(e.counterpart_transform(side,d,0.))
                o,i=(other,fixed["tube_a"]) if side=="a" else (fixed["tube_b"],other)
                screw=source.copy()
                screw.apply_transform(l.screw_transform(side,d,candidate["offset_mm"],candidate["phase_deg"]))
                overlaps=[a.overlap(o,screw),a.overlap(i,screw),a.overlap(o,i)]
                samples=a.restraint(i,screw)
                blocked=all(any(r["shift_mm"]*sign>0 and abs(r["shift_mm"])<=2.5 and r["screw_overlap_mm3"]>LIMIT for r in samples) for sign in (-1,1))
                checks.append({"side":side,"index":idx,"overlaps_mm3":overlaps,"shifts":samples,"pass":max(overlaps)<=LIMIT and blocked})
        verification.append({"offset_mm":candidate["offset_mm"],"phase_deg":candidate["phase_deg"],"checks":checks})
        if all(r["pass"] for r in checks):
            accepted=candidate
            break
    # Folded screen: keep validated screw candidate, or known clear retracted pose.
    lock=accepted or {"offset_mm":18.,"phase_deg":0.}
    base=e.m.parts(archive,180.,0.)
    rotation=tm.transformations.rotation_matrix(np.pi,(0,0,1))
    folded=[]
    for iax in (0,3,6,9,12):
        for ibx in (0,3,6,9,12):
            parts=dict(base)
            for side,idx,src in (("a",iax,original_outer),("b",ibx,original_inner)):
                d=l.indexed_depth(idx)
                body=src.copy()
                body.apply_transform(e.counterpart_transform(side,d,0.))
                screw=source.copy()
                screw.apply_transform(l.screw_transform(side,d,lock["offset_mm"],lock["phase_deg"]))
                if side=="b":
                    body.apply_transform(rotation)
                    screw.apply_transform(rotation)
                parts["distal_"+side]=body
                parts["extension_screw_"+side]=screw
            defects=ia.audit(parts)
            folded.append({"index_a":iax,"index_b":ibx,"collisions":defects,"clear":not defects})
        print("folded index A",iax,flush=True)
    dest=Path(destination)
    dest.mkdir(parents=True,exist_ok=True)
    result={"status":"discrete_phase_and_folded_screen_only","phase_rows":rows,"verification":verification,"accepted_lock_candidate":accepted,"folded_lock_pose":lock,"folded":folded,"limitations":"No continuous helical insertion, retention, physical load or swept folding path verified. A clear folded sample is not a safe folding path."}
    (dest/"report.json").write_text(json.dumps(result,indent=2),encoding="utf-8")
    return result
