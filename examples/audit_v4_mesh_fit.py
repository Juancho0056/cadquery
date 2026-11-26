import argparse,json
from modular_arm.v4.mesh_fit import fit_report
p=argparse.ArgumentParser()
p.add_argument("archive")
p.add_argument("--angle",type=float,default=0.)
p.add_argument("--travel",type=float,default=0.)
a=p.parse_args()
r=fit_report(a.archive,a.angle,a.travel)
print(json.dumps(r,indent=2))
if any(x["surface_contacts"] for x in r):
    print("FAIL: surface contacts detected; review alignment and geometry")
    raise SystemExit(1)
print("No surface contacts detected; containment and fit not certified")
