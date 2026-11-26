import json
from modular_arm.v4.assembly_screen import report
r=report()
print(json.dumps(r,indent=2))
raise SystemExit(1 if r["issues"] else 0)
