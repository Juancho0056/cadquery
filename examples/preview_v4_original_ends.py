import argparse,json
from modular_arm.v4.reference_preview import export_preview,terminal_report
p=argparse.ArgumentParser()
p.add_argument("archive")
p.add_argument("--output",default="out/v4_reference_preview")
p.add_argument("--angle",type=float,default=0)
p.add_argument("--travel",type=float,default=0)
p.add_argument('--rigid',action='store_true')
a=p.parse_args()
print(json.dumps(terminal_report(a.archive),indent=2))
for path in export_preview(a.archive,a.output,a.angle,a.travel,rigid=a.rigid):
    print(path)
print("PREVIEW ONLY: no mesh collision or physical compatibility certificate")
