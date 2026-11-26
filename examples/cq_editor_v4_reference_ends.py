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
from modular_arm.v4.reference_ends import parts,pin_gauge,interference_report
ANGLE=0.
TRAVEL=0.
p=parts(ANGLE,TRAVEL)
p["pin_a"]=pin_gauge(False)
p["pin_b"]=pin_gauge(True,ANGLE)
for name,s in p.items():
    show_object(s,name=name,options={"color":(.15,.65,.55) if name=="arm_b" else (.9,.55,.15) if name=="button" else (.55,.65,.75)})
print("Interface interference audit:",interference_report(ANGLE,TRAVEL))
