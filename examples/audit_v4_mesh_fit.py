import argparse,json
from modular_arm.v4.mesh_fit import fit_report,failures
p=argparse.ArgumentParser()
p.add_argument("archive")
p.add_argument("--angle",type=float,default=0.)
p.add_argument("--travel",type=float,default=0.)
a=p.parse_args()
r=fit_report(a.archive,a.angle,a.travel)
print(json.dumps(r,indent=2))
issues=failures(r)
if issues:
    print(json.dumps(issues,indent=2))
    raise SystemExit(1)
print("PASS: sampled surface/containment/proximity screen only; no mechanical fit certificate")
