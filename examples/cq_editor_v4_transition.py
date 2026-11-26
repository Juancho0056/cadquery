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
    raise RuntimeError("Set MODULAR_ARM_ROOT")
from modular_arm.v4 import joint
from modular_arm.v4.transition import finished_half
from itertools import combinations
ANGLE=0.
TRAVEL=0.
p=joint.parts(ANGLE,TRAVEL)
p["arm_a"]=finished_half(1)
p["arm_b"]=joint.turn(finished_half(5),ANGLE)
for name,s in p.items():
    show_object(s,name=name,options={"color":(.15,.65,.55) if name=="arm_b" else (.9,.55,.15) if name=="button" else (.55,.65,.75)})
errors=[]
for (a,s),(b,t) in combinations(p.items(),2):
    q=s.intersect(t)
    if q.Volume()>1e-6:
        errors.append((a,b,q.Volume()))
        show_object(q,name="interference_"+a+"_"+b,options={"color":(1.,0.,0.)})
print("CAD interference audit:",errors if errors else "PASS")
