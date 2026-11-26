"""Geometric return-spring proposal, with explicit rest and deflected shapes.

The prescribed curve is not an elastic solution. No PETG force or lifetime is certified.
"""
from dataclasses import dataclass
import math
import cadquery as cq

@dataclass(frozen=True,slots=True)
class ReturnConfig:
    length_mm: float=25.
    width_mm: float=1.2
    height_mm: float=4.
    anchor_length_mm: float=6.
    preload_mm: float=.3
    mounting_clearance_mm: float=.4

    def __post_init__(self):
        if not all(math.isfinite(getattr(self,n)) and getattr(self,n)>0 for n in self.__dataclass_fields__):
            raise ValueError("positive finite dimensions required")
        if self.anchor_length_mm>=self.length_mm or self.width_mm>=self.length_mm/5:
            raise ValueError("invalid flexure dimensions")

def return_deflection(c,r,q):
    if not math.isfinite(q) or not 0<=q<=20:
        raise ValueError("return angle outside prototype")
    return c.preload_mm+r*math.sin(math.radians(q))

def beam_strain_estimate(c,deflection):
    """Small-deflection cantilever screening only; not a material qualification."""
    return 1.5*c.width_mm*deflection/c.length_mm**2

def return_leaf(c,*,deflection=0.):
    """Local coordinates: root at y=0; free tip at y=L, bend along +X."""
    if not math.isfinite(deflection) or not 0<=deflection<c.length_mm/2:
        raise ValueError("invalid prescribed deflection")
    pts=[]
    for sign,indices in ((-1,range(25)),(1,range(24,-1,-1))):
        for i in indices:
            u=i/24
            x=deflection*u*u*(3-2*u)
            pts.append((x+sign*c.width_mm/2,c.length_mm*u))
    leaf=cq.Workplane("XY").polyline(pts).close().extrude(c.height_mm)
    root=cq.Workplane("XY").box(4*c.width_mm,c.anchor_length_mm,c.height_mm,
             centered=(True,False,False)).translate((0,-c.anchor_length_mm,0))
    return leaf.union(root).val()
