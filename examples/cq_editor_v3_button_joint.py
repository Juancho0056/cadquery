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
from modular_arm.v3.button_joint import parts,Joint,spring_envelope
import cadquery as cq
from itertools import combinations
ANGLE=60.
PRESSED=False
EXPLODED=False
SHOW_SPRING_ENVELOPE=False
p=parts(angle=ANGLE,pressed=PRESSED)
colors={"fork":(.55,.65,.75),"rotor":(.15,.65,.55),"button_dog":(.9,.55,.15),"bottom_cap":(.5,.5,.5)}
for i,(name,s) in enumerate(p.items()):
    show_object(s.translate(cq.Vector(0,0,15*i)) if EXPLODED else s,name=name,options={"color":colors[name]})
if SHOW_SPRING_ENVELOPE:
    show_object(spring_envelope(Joint(),PRESSED),name="reserved_spring_volume",options={"color":(.8,.2,.6),"alpha":.3})
for (a,s),(b,t) in combinations(p.items(),2):
    overlap=s.intersect(t)
    if overlap.Volume()>1e-6:
        print("INTERFERENCE",a,b,overlap.Volume())
        if not EXPLODED:
            show_object(overlap,name="overlap_"+a+"_"+b,options={"color":(1.,0.,0.)})
