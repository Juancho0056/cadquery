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
from modular_arm.assembly.ratchet import ratchet_parts

# Prescribed snapshots, not a physical selector.
POSE="B"
EXPLODED=False
SHOW_FORK=True
poses={"B":(0,0,0,True),"A":(-9,20,7.5,False),"C":(9,7.5,20,False)}
angle,lower,upper,lock=poses[POSE]
colors={"fork":(0.35,0.43,0.52),"rotor":(0.87,0.68,0.23),"lower_pawl":(0.92,0.46,0.12),"upper_pawl":(0.30,0.62,0.85),"angular_lock":(0.85,0.16,0.16)}
for n,(name,part) in enumerate(ratchet_parts(angle_deg=angle,lower_release=lower,upper_release=upper,lock_inserted=lock).items()):
    if name=="fork" and not SHOW_FORK:
        continue
    if EXPLODED:
        part=part.translate(cq.Vector(0,0,n*12))
    show_object(part,name=name,options={"color":colors.get(name,(0.7,0.76,0.81))})
