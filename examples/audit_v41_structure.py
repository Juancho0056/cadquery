import argparse,json
from pathlib import Path
from modular_arm.v4.modular_structural_screen import report
p=argparse.ArgumentParser()
p.add_argument("--output",default="out/v41_structural_screen.json")
a=p.parse_args()
r=report()
dest=Path(a.output)
dest.parent.mkdir(parents=True,exist_ok=True)
dest.write_text(json.dumps(r,indent=2),encoding="utf-8")
for label,h in r["halves"].items():
    print("ARM",label.upper())
    for component in ("head","tenon"):
        print(component)
        for metric,row in h[component+"_minima"].items():
            print(f'  {metric}: {row[metric]:.3f} at x={row["x_mm"]:.2f}')
    for row in h["sections"]:
        if row["station_mm"]==38.:
            print("  bore center area fraction:",row["tenon_area_fraction"])
            print("  bore center section modulus fractions:",row["tenon_wy_fraction"],row["tenon_wz_fraction"])
print("Report:",dest)
