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
from modular_arm.v3.modular_elbow import parts
from modular_arm.v3.assembly import fasteners
from itertools import combinations
ANGLE=45.
TRAVEL=0.
CONNECTED=False
p=parts(ANGLE,TRAVEL,connected=CONNECTED)
for name,s in p.items():
    show_object(s,name=name,options={"color":(.15,.65,.55) if name=="rotating" else (.9,.55,.15) if name=="plunger" else (.55,.65,.75)})
for name,s in fasteners().items():
    show_object(s,name=name,options={"color":(.35,.35,.4)})
errors=[]
for (a,s),(b,t) in combinations(p.items(),2):
    overlap=s.intersect(t)
    if overlap.Volume()>1e-6:
        errors.append((a,b,overlap.Volume()))
        show_object(overlap,name="overlap_"+a+"_"+b,options={"color":(1.,0.,0.)})
print("Interference audit:",errors if errors else "PASS")
