"""State contract only. Does not implement a selector or directional motion."""
from enum import Enum

class Mode(str,Enum):
    A="clockwise_allowed"
    B="both_blocked"
    C="counterclockwise_allowed"

def requested_direction_allowed(mode: Mode, clockwise: bool, safety_lock_inserted: bool) -> bool:
    if not isinstance(mode,Mode):
        raise ValueError("unknown mode")
    if safety_lock_inserted or mode is Mode.B:
        return False
    return clockwise if mode is Mode.A else not clockwise
