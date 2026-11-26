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
for name,part in elbow_parts(angle=ANGLE,locked=LOCKED).items():
    show_object(part,name=name,options={"color":(.85,.55,.15) if "pin" in name else (.15,.65,.55) if name=="moving" else (.55,.65,.75)})
