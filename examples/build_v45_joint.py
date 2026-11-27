import argparse,json
from modular_arm.v45.export import export
p=argparse.ArgumentParser()
p.add_argument("--output",default="out/v452_joint")
a=p.parse_args()
print(json.dumps(export(a.output),indent=2))
