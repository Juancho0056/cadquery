import argparse,json
from modular_arm.v4.fold_diagnostic import export_diagnostic
p=argparse.ArgumentParser()
p.add_argument("archive")
p.add_argument("--output",default="out/v4_fold_diagnostic")
p.add_argument('--clocking',type=float,default=0.)
a=p.parse_args()
print(json.dumps(export_diagnostic(a.archive,a.output,clocking=a.clocking),indent=2))
