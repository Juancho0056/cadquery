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

from modular_arm.mechanisms.selector_contract import Mode
from modular_arm.mechanisms.selector_slider import selector_bench
MODE=Mode.B
for name,part in selector_bench(mode=MODE).items():
    show_object(part,name=name,options={"color":(.23,.55,.8) if name=="selector_slider" else (.15,.65,.55)})
