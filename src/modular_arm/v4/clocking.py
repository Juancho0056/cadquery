"""Common rigid longitudinal rotation for module members in a shared frame."""
import math
import numpy as np

def validate_clocking(angle):
    if not math.isfinite(angle) or angle not in (0.,90.,180.,270.):
        raise ValueError("clocking must be 0, 90, 180 or 270 degrees")

def matrix(angle):
    validate_clocking(angle)
    # Exact cardinal rotations avoid near-coplanar boolean artifacts.
    c,s={0.:(1,0),90.:(0,1),180.:(-1,0),270.:(0,-1)}[angle]
    return np.array([[1,0,0,0],[0,c,-s,0],[0,s,c,0],[0,0,0,1]],dtype=float)

def rotate_module(mesh,angle):
    """Apply to every member already registered in the same local X frame."""
    result=mesh.copy()
    if angle:
        result.apply_transform(matrix(angle))
    else:
        validate_clocking(angle)
    return result
