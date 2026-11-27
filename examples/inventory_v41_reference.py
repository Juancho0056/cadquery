"""Inventory original parts locally; no inferred assembly or redistribution."""
import argparse,json,hashlib
from pathlib import Path
import numpy as np
from modular_arm.v4.reference_arm import read_part

def inventory(archive):
    rows=[]
    for i in range(1,41):
        p=read_part(archive,i)
        lo,hi=p.bounds
        span=hi-lo
        axis=int(np.argmax(span))
        profiles=[]
        for fraction in (.05,.15,.30,.50,.70,.85,.95):
            # Sample transverse sections along each part's longest bounding axis.
            station=float(lo[axis]+fraction*span[axis])
            tri=p.triangles.astype(float)
            hits=[]
            for j in range(3):
                a,b=tri[:,j],tri[:,(j+1)%3]
                mask=(a[:,axis]-station)*(b[:,axis]-station)<0
                a,b=a[mask],b[mask]
                hits.append(a+(b-a)*((station-a[:,axis])/(b[:,axis]-a[:,axis]))[:,None])
            xy=np.delete(np.concatenate(hits),axis,axis=1)
            if len(xy):
                center=(xy.max(0)+xy.min(0))/2
                r=np.linalg.norm(xy-center,axis=1)
                profiles.append({"fraction":fraction,"station_mm":station,"center":center.tolist(),
                                 "width_mm":(xy.max(0)-xy.min(0)).tolist(),
                                 "radial_quantiles_mm":np.quantile(r,[0,.1,.5,.9,1]).tolist()})
        rows.append({"index":i,"name":p.name,"triangles":len(p.triangles),
                     "bounds_mm":[lo.tolist(),hi.tolist()],"size_mm":span.tolist(),
                     "long_axis":"XYZ"[axis],"sections":profiles,
                     "role":"existing_v41_body" if i in (1,5) else "unassigned"})
    return {"status":"inventory_only_not_assembly","archive_sha256":hashlib.sha256(Path(archive).read_bytes()).hexdigest(),"parts":rows}

if __name__=="__main__":
    p=argparse.ArgumentParser()
    p.add_argument("archive")
    p.add_argument("--output",default="out/v41_reference_inventory.json")
    a=p.parse_args()
    r=inventory(a.archive)
    dest=Path(a.output)
    dest.parent.mkdir(parents=True,exist_ok=True)
    dest.write_text(json.dumps(r,indent=2),encoding="utf-8")
    for row in r["parts"]:
        print(f'{row["index"]:02} {row["name"]} | '+ " x ".join(f"{x:.2f}" for x in row["size_mm"])+f' | {row["long_axis"]}')
    print("Report:",dest)
