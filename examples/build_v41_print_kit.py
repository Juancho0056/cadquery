import argparse,json
from modular_arm.v4.print_kit import export
p=argparse.ArgumentParser()
p.add_argument("archive")
p.add_argument("--segments",type=int,default=2)
p.add_argument("--output",default="out/v41_print_kit")
a=p.parse_args()
print(json.dumps(export(a.archive,a.output,a.segments),indent=2))
