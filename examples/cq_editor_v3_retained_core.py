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
from modular_arm.v3.locking_core import Core,spring_space
from modular_arm.v3.retained_core import parts
import cadquery as cq
from itertools import combinations
ANGLE=0.
TRAVEL=0.
EXPLODED=False
SHOW_SPRING_SPACE=False
p=parts(angle=ANGLE,travel=TRAVEL)
colors={"stationary":(.55,.65,.75),"rotating":(.15,.65,.55),"plunger":(.9,.55,.15),"keeper":(.45,.45,.5)}
for i,(name,s) in enumerate(p.items()):
    show_object(s.translate(cq.Vector(0,0,25*i)) if EXPLODED else s,name=name,options={"color":colors[name]})
if SHOW_SPRING_SPACE and not EXPLODED:
    show_object(spring_space(Core(),TRAVEL),name="reserved_spring_volume",options={"color":(.8,.2,.6),"alpha":.3})
defects=[]
for (a,s),(b,t) in combinations(p.items(),2):
    overlap=s.intersect(t)
    if overlap.Volume()>1e-6:
        defects.append((a,b,overlap.Volume()))
        if not EXPLODED:
            show_object(overlap,name="overlap_"+a+"_"+b,options={"color":(1.,0.,0.)})
print("Interference audit:",defects if defects else "PASS")
