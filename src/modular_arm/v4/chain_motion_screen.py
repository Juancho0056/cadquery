"""Configuration-dependent sampled motion screen, not a mechanical stop."""
import json
from pathlib import Path
from modular_arm.v4 import alternating_chain as a
from modular_arm.v4 import modular_chain as c

def sweep(archive,destination):
    dest=Path(destination)
    dest.mkdir(parents=True,exist_ok=True)
    source=a.templates(archive)
    rows=[]
    # Check adjacent and remote collisions while moving one joint at a time.
    # Stop at first detected collision; never classify later free samples as reachable.
    for indices in ((6,6,6),(6,6,6,6),(6,0,12,6),(6,12,0,6)):
        n=len(indices)-1
        for moving in range(n):
            for sign in (-1,1):
                samples=[]
                for magnitude in range(0,181,15):
                    angles=[0.]*n
                    travels=[0.]*n
                    angles[moving]=float(sign*magnitude)
                    travels[moving]=7.
                    parts,_=a.assemble(archive,angles,indices,source=source,travels=travels)
                    defects=c.ia.audit(parts)
                    sample={"angle_deg":angles[moving],"collisions":defects,"clear":not defects}
                    samples.append(sample)
                    if defects: break
                clear=[r["angle_deg"] for r in samples if r["clear"]]
                row={"indices":indices,"moving_joint":moving,"direction":sign,
                     "last_clear_sample_deg":clear[-1] if clear else None,
                     "first_collision_sample_deg":samples[-1]["angle_deg"] if not samples[-1]["clear"] else None,
                     "samples":samples}
                rows.append(row)
                print(json.dumps(row),flush=True)
    result={"status":"sampled_motion_only","rows":rows,
            "limitations":"One joint at a time from straight, released button; 15 degree sampling can miss collisions. Not a safe angle limit or physical stop. No simultaneous motion or load qualification."}
    (dest/"report.json").write_text(json.dumps(result,indent=2),encoding="utf-8")
    return result
