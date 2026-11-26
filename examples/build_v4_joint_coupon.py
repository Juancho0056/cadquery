import argparse,json
from pathlib import Path
from modular_arm.v4.prototype_coupon import printable_parts
p=argparse.ArgumentParser()
p.add_argument("--output",default="out/v4_joint_coupon")
a=p.parse_args()
dest=Path(a.output)
dest.mkdir(parents=True,exist_ok=True)
rows={}
for name,s in printable_parts().items():
    s.exportStl(str(dest/(name+".stl")),tolerance=.05,angularTolerance=.08)
    bb=s.BoundingBox()
    rows[name]={"solid_volume_mm3":s.Volume(),"bounds_mm":[bb.xlen,bb.ylen,bb.zlen]}
(dest/"manifest.json").write_text(json.dumps({"status":"unloaded_fit_test_only","parts":rows,"hardware":"One compatible compression spring and three qualified metal retainers; not included."},indent=2),encoding="utf-8")
print(json.dumps(rows,indent=2))
