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
from modular_arm.v4.joint import parts,spring_space
import cadquery as cq
from itertools import combinations
ANGLE=45.
TRAVEL=0.
EXPLODED=False
SHOW_SPRING_SPACE=False
p=parts(ANGLE,TRAVEL,hardware=not EXPLODED)
colors={"arm_a":(.55,.65,.75),"arm_b":(.15,.65,.55),"button":(.9,.55,.15)}
offset={"arm_a":0,"button":30,"arm_b":65}
for name,s in p.items():
    show_object(s.translate(cq.Vector(0,0,offset[name])) if EXPLODED else s,name=name,options={"color":colors.get(name,(.4,.4,.4))})
if SHOW_SPRING_SPACE and not EXPLODED:
    show_object(spring_space(TRAVEL),name="spring_envelope",options={"alpha":.3})
for (a,s),(b,t) in combinations(p.items(),2):
    overlap=s.intersect(t)
    if overlap.Volume()>1e-6:
        print("INTERFERENCE",a,b,overlap.Volume())
