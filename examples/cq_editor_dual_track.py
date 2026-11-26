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
from modular_arm.mechanisms.dual_track import DualTrackConfig,bench_parts
c=DualTrackConfig()
ANGLE=0.
EXPLODED=False
# Retract both coupons for a clear inspection; change offsets only for contact study.
OFFSET=c.tip_radius_mm-c.root_radius_mm+c.follower_width_mm
for n,(name,part) in enumerate(bench_parts(c,angle_deg=ANGLE,upper_offset_mm=OFFSET,lower_offset_mm=OFFSET).items()):
    if EXPLODED:
        part=part.translate(cq.Vector(0,0,n*15))
    show_object(part,name=name,options={"color":(0.87,0.68,0.23) if name=="rotor" else (0.92,0.46,0.12)})
