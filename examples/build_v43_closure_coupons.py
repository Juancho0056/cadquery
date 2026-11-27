import argparse,json
from modular_arm.v43.closure_coupons import export
p=argparse.ArgumentParser()
p.add_argument("--output",default="out/v43_closure_coupons")
a=p.parse_args()
print(json.dumps(export(a.output),indent=2))
