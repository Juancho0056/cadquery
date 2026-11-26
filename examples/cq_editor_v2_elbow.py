import os,sys
from pathlib import Path
for start in [Path(os.environ.get("MODULAR_ARM_ROOT",Path.cwd())),Path(globals().get("__file__",Path.cwd())).resolve().parent,Path.cwd()]:
    for root in (start,*start.parents):
        if (root/"src"/"modular_arm"/"__init__.py").is_file():
            sys.path.insert(0,str(root/"src"))
            break
    else:
        continue
    break
else:
    raise RuntimeError("Set MODULAR_ARM_ROOT to repository root")
from modular_arm.v2.elbow import elbow_parts
ANGLE=60.
LOCKED=True
from itertools import combinations
import cadquery as cq
EXPLODED=False
CHECK_INTERFERENCE=True
CAPTIVE=True
parts=elbow_parts(angle=ANGLE,locked=LOCKED,captive=CAPTIVE)
for i,(name,part) in enumerate(parts.items()):
    shown=part.translate(cq.Vector(0,0,12*i)) if EXPLODED else part
    show_object(shown,name=name,options={"color":(.85,.55,.15) if "pin" in name or "keeper" in name else (.15,.65,.55) if name=="moving" else (.55,.65,.75)})
if CHECK_INTERFERENCE:
    defects=[]
    for (a,p),(b,q) in combinations(parts.items(),2):
        overlap=p.intersect(q)
        if overlap.Volume()>1e-6:
            defects.append((a,b,overlap.Volume()))
            if not EXPLODED:
                show_object(overlap,name=f"interference_{a}_{b}",options={"color":(1.,0.,0.)})
    print("Interference audit:",defects if defects else "PASS")
