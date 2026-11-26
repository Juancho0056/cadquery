import json
from modular_arm.v4.hardware_screen import report
r=report()
print(json.dumps(r,indent=2))
if any(v>1e-6 for row in r["poses"] for k,v in row.items() if "intersection" in k):
    raise SystemExit(1)
if any(p["intersection_a_mm3"]>1e-6 or p["intersection_b_mm3"]>1e-6 or p["axial_capture_test_mm3"]<=0 for p in r["retainers"]["checks"]):
    raise SystemExit(1)
