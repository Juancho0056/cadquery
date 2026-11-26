import argparse,json
from modular_arm.v4.modular_connection import export
p=argparse.ArgumentParser()
p.add_argument("--archive",help="Original STL ZIP; omit for short connection coupons")
p.add_argument("--output",default="out/v41_modular")
p.add_argument("--angle",type=float,default=0.)
p.add_argument("--travel",type=float,default=0.)
a=p.parse_args()
print(json.dumps(export(a.output,a.archive,a.angle,a.travel),indent=2))
