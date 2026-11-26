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
from modular_arm.v2.connector import assembly
LENGTH=100.
EXPLODED=False
for name,part in assembly(LENGTH,withdrawal=30. if EXPLODED else 0.,locked=not EXPLODED).items():
    show_object(part,name=name,options={"color":(.15,.65,.55) if name=="segment" else (.85,.55,.15) if name.startswith("pin") else (.55,.65,.75)})
