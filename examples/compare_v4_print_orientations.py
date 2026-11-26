import argparse,json
from pathlib import Path
from modular_arm.v4.integrated_arm import parts
from modular_arm.v4.orientation_screen import compare
p=argparse.ArgumentParser()
p.add_argument("archive")
p.add_argument("--limit",type=float,default=45.)
p.add_argument("--output",default="out/v4_orientation_comparison.json")
a=p.parse_args()
m=parts(a.archive,flush=True,clocking=180.)
r={name:compare(m[name].triangles,direction,a.limit)
   for name,direction in (("arm_a_integrated",-1),("arm_b_integrated",1))}
dest=Path(a.output)
dest.parent.mkdir(parents=True,exist_ok=True)
dest.write_text(json.dumps(r,indent=2),encoding="utf-8")
for name,rows in r.items():
    print(name)
    for row in sorted(rows,key=lambda x:x["unsupported_candidate_area_mm2"]):
        print(f'{row["name"]:20} candidate={row["unsupported_candidate_area_mm2"]:9.2f} mm2  bed_proxy={row["bed_face_area_mm2"]:9.2f} mm2  height={row["height_mm"]:7.2f} mm')
