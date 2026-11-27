"""Local closure-only fit coupon, same dimensions and cutters as V4.5.2."""
import argparse,json
from pathlib import Path
import cadquery as cq
from modular_arm.v45 import joint as j

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--output",default="out/v452_bayonet_coupon")
    args=ap.parse_args()
    d=Path(args.output); d.mkdir(parents=True,exist_ok=True)
    for old in (*d.glob("*.stl"),*d.glob("*.step")): old.unlink()
    # Preserve actual housing wall, entrances, guide slots and seam key.
    b=j.ring(j.R,j.POCKET,33.9,7.5)
    b=j.bayonet_tracks(b)
    for a in (90.,270.): b=b.cut(j.guide(a,33.8,8.,.2))
    b,c,k=j.lock_pair(b,j.cap())
    poses=[(-j.LOCK_ANGLE,z) for z in (12.,9.,6.,4.,3.,2.,1.,.5,0.)]+[(a,0.) for a in (-25.,-20.,-15.,-10.,-5.,0.)]
    rows=[{"angle":a,"lift":z,"overlap_mm3":j.overlap(b,j.turn(c,a).translate(j.V(0,0,z)))} for a,z in poses]
    checks={"seated_key_clear":j.overlap(k,b)+j.overlap(k,c),
            "locked_pull":j.overlap(b,c.translate(j.V(0,0,1.))),
            "unlocked_pull_clear":j.overlap(b,j.turn(c,-j.LOCK_ANGLE).translate(j.V(0,0,2.))),
            "end_stop":j.overlap(b,j.turn(c,5.)),
            "key_minus":j.overlap(k,j.turn(c,-5.)),
            "key_plus":j.overlap(k,j.turn(c,5.))}
    passed=max(x["overlap_mm3"] for x in rows)<=j.TOL and all(checks[n]<=j.TOL for n in ("seated_key_clear","unlocked_pull_clear")) and all(checks[n]>j.TOL for n in ("locked_pull","end_stop","key_minus","key_plus"))
    report={"passed":passed,"poses":rows,"checks":checks,"status":"closure_only_no_load_rating"}
    (d/"report.json").write_text(json.dumps(report,indent=2))
    if passed:
        for n,s in (("body",b),("cap",c.rotate((0,0,0),(1,0,0),180)),("key",k)):
            s=s.translate(j.V(0,0,-s.BoundingBox().zmin))
            cq.exporters.export(s,str(d/(n+".stl")),tolerance=.04,angularTolerance=.1)
    print(json.dumps(report,indent=2))
if __name__=="__main__": main()
