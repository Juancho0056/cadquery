import argparse,json
from modular_arm.v4.fold_diagnostic import export_diagnostic
p=argparse.ArgumentParser()
p.add_argument("archive")
p.add_argument("--output",default="out/v4_fold_diagnostic")
a=p.parse_args()
print(json.dumps(export_diagnostic(a.archive,a.output),indent=2))
