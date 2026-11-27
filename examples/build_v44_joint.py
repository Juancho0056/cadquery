import argparse,json
from modular_arm.v44.export import export
p=argparse.ArgumentParser()
p.add_argument("--output",default="out/v44_joint")
a=p.parse_args()
print(json.dumps(export(a.output),indent=2))
