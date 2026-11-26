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
from modular_arm.assembly.common_selector import common_selector_parts
from modular_arm.mechanisms.selector_contract import Mode
MODE=Mode.B
SHOW_LID=True
EXPLODED=False
INDEX_INSERTED=True
SHOW_ARMS=True
SHOW_STOPS=True
SHOW_EXTENSIONS=True
ROTOR_ANGLE=0.0
ACTIVE_RELEASE=0.0
LOCK_INSERTED=True
colors={"housing":(.35,.43,.52),"lid":(.46,.56,.65),"rotor":(.87,.68,.23),"moving_extension":(.87,.68,.23),"fixed_extension":(.35,.43,.52),"common_selector":(.15,.65,.55),"angular_lock":(.85,.16,.16),"angular_lock_grooved_pin":(.85,.16,.16),"selector_index_pin":(.82,.38,.12)}
for i,(name,part) in enumerate(common_selector_parts(mode=MODE,index_inserted=INDEX_INSERTED,include_arms=SHOW_ARMS,rotor_angle=ROTOR_ANGLE,lock_inserted=LOCK_INSERTED,active_release=ACTIVE_RELEASE,include_stops=SHOW_ARMS and SHOW_STOPS,include_extensions=SHOW_ARMS and SHOW_EXTENSIONS).items()):
    if name=="lid" and not SHOW_LID:
        continue
    if EXPLODED:
        part=part.translate(cq.Vector(0,0,i*10))
    show_object(part,name=name,options={"color":colors.get(name,(.15,.65,.55) if "return_leaf" in name else (.25,.55,.85) if "actuator" in name else (.7,.76,.81))})
