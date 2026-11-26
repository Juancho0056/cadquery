import argparse,json
from pathlib import Path
import numpy as np
from modular_arm.v4.integrated_arm import parts
from modular_arm.v4.printability import overhang_report,regional_overhang
from modular_arm.v4.reference_preview import write_stl
p=argparse.ArgumentParser()
p.add_argument("archive")
p.add_argument("--clocking",type=float,default=180.)
p.add_argument("--limit",type=float,default=45.)
p.add_argument("--output",default="out/v4_printability_diagnostic")
a=p.parse_args()
m=parts(a.archive,flush=True,clocking=a.clocking)
dest=Path(a.output)
dest.mkdir(parents=True,exist_ok=True)
report={}
for n,axis in (("arm_a_integrated",(1,0,0)),("arm_b_integrated",(-1,0,0))):
    r=overhang_report(m[n].triangles,axis,a.limit)
    ids=r.pop("candidate_indices")
    r["regions"]=regional_overhang(m[n].triangles,ids,direction=-1 if n=="arm_a_integrated" else 1)
    r["build_axis"]=list(axis)
    path=dest/(n+"_OVERHANG_SURFACES_NOT_PRINTABLE.stl")
    if ids:
        write_stl(path,m[n].triangles[np.asarray(ids,dtype=int)])
    else:
        path.unlink(missing_ok=True)
    m[n].export(dest/(n+".stl"))
    report[n]=r
(dest/"report.json").write_text(json.dumps(report,indent=2),encoding="utf-8")
print(json.dumps(report,indent=2))
