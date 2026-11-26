import os
import sys
from pathlib import Path

# CQ-editor may execute a string without a usable __file__.
_candidates = []
if os.environ.get("MODULAR_ARM_ROOT"):
    _candidates.append(Path(os.environ["MODULAR_ARM_ROOT"]))
if globals().get("__file__"):
    _candidates.append(Path(__file__).resolve().parent)
_candidates.append(Path.cwd())
for _start in _candidates:
    for _root in (_start, *_start.parents):
        if (_root / "src" / "modular_arm" / "__init__.py").is_file():
            sys.path.insert(0, str(_root / "src"))
            break
    else:
        continue
    break
else:
    raise RuntimeError("Set MODULAR_ARM_ROOT to the cloned cadquery repository and restart CQ-editor.")

import cadquery as cq
from modular_arm.assembly.serviceable_ratchet import serviceable_parts
EXPLODED=False
SHOW_LID=True
colors={"housing":(.35,.43,.52),"lid":(.46,.56,.65),"rotor":(.87,.68,.23),"lower_pawl":(.92,.46,.12),"upper_pawl":(.30,.62,.85),"angular_lock":(.85,.16,.16)}
for n,(name,part) in enumerate(serviceable_parts().items()):
    if name=="lid" and not SHOW_LID:
        continue
    if EXPLODED:
        part=part.translate(cq.Vector(0,0,n*12))
    show_object(part,name=name,options={"color":colors.get(name,(.7,.76,.81))})
