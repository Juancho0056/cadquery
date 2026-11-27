import argparse,json
from modular_arm.v45.export import export
p=argparse.ArgumentParser()
p.add_argument("--output")
p.add_argument("--return-mode",choices=("magnet","spring"),default="magnet")
a=p.parse_args()
print(json.dumps(export(a.output or ("out/v453_joint_"+a.return_mode),a.return_mode),indent=2))
