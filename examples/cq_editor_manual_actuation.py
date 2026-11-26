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

from modular_arm.assembly.manual_actuation import manual_parts
from modular_arm.mechanisms.selector_contract import Mode
MODE=Mode.B
SHOW_LID=False
for name,part in manual_parts(mode=MODE).items():
    if name=="lid" and not SHOW_LID:
        continue
    color=(.25,.55,.85) if "actuator" in name else (.87,.68,.23) if name=="rotor" else (.65,.72,.78)
    show_object(part,name=name,options={"color":color})
