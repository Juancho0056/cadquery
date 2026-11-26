import argparse,json
from modular_arm.v4.integrated_arm import parts
from modular_arm.v4.printability import overhang_report
p=argparse.ArgumentParser()
p.add_argument("archive")
p.add_argument("--clocking",type=float,default=180.)
p.add_argument("--limit",type=float,default=45.)
a=p.parse_args()
m=parts(a.archive,flush=True,clocking=a.clocking)
print(json.dumps({n:overhang_report(m[n].triangles,axis,a.limit)
 for n,axis in (("arm_a_integrated",(1,0,0)),("arm_b_integrated",(-1,0,0)))},indent=2))
