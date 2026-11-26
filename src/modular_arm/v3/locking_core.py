"""Concentric locking-core bench, independently designed from reference principle.
All units mm. Prescribed rigid poses; no load rating or automatic return claimed.
"""
from dataclasses import dataclass
import math
import cadquery as cq

@dataclass(frozen=True,slots=True)
class Core:
    root_radius: float=14.
    tip_radius: float=18.
    clearance: float=.4
    stem_radius: float=11.
    housing_radius: float=30.
    stroke: float=7.
    engagement: float=6.
    spring_od: float=18.
    spring_free: float=23.
    spring_solid: float=5.
    spring_rate: float=.8
    def __post_init__(self):
        if not all(math.isfinite(getattr(self,n)) and getattr(self,n)>0 for n in self.__dataclass_fields__):
            raise ValueError("positive finite dimensions required")
        if self.tip_radius-self.root_radius<4 or self.root_radius<14 or self.stem_radius>=self.root_radius:
            raise ValueError("minimum tooth section or stem violated")
        if self.housing_radius-self.tip_radius-self.clearance<5 or self.stroke<self.engagement+.5:
            raise ValueError("insufficient wall or release stroke")
        if self.stroke!=7 or self.engagement!=6:
            raise ValueError("unsupported axial stack")
        if not self.spring_solid<13-self.stroke<13<self.spring_free or self.spring_od/2>=self.root_radius:
            raise ValueError("spring envelope invalid")

def cyl(r,z,h):
    return cq.Solid.makeCylinder(r,h,cq.Vector(0,0,z))

def profile_points(c,clearance=0.):
    # Eight broad lobes. Flat angular crest and smooth radial transitions,
    # sampled polygon; no guaranteed physical fillet radius.
    result=[]
    for i in range(256):
        a=2*math.pi*i/256
        d=abs((math.degrees(a)+22.5)%45-22.5)
        t=max(0.,min(1.,(17.-d)/9.))
        r=c.root_radius+(c.tip_radius-c.root_radius)*t*t*(3-2*t)+clearance
        result.append((r*math.cos(a),r*math.sin(a)))
    return result

def profile(c,z,h,clearance=0.):
    return cq.Workplane("XY").polyline(profile_points(c,clearance)).close().extrude(h).translate((0,0,z)).val()

def stationary(c):
    # Integral cup floor, round spring chamber and stationary toothed guide.
    p=cyl(c.housing_radius,-10,22)
    p=p.cut(cyl(c.tip_radius+c.clearance+1,-7,7.1))
    return p.cut(profile(c,0,13,c.clearance)).clean()

def rotating(c):
    return cyl(c.housing_radius,12.4,6).cut(profile(c,12,7,c.clearance)).clean()

def plunger(c,travel=0.):
    if not math.isfinite(travel) or not 0<=travel<=c.stroke:
        raise ValueError("invalid button travel")
    p=profile(c,6,12.4).fuse(cyl(c.stem_radius,18.3,12.1)).clean()
    return p.translate(cq.Vector(0,0,-travel))

def spring_space(c,travel=0.):
    # Envelope between integral floor and plunger underside; NOT a modeled spring.
    return cyl(c.spring_od/2,-7,13-travel)

def spring_force_estimate(c,travel=0.):
    if not 0<=travel<=c.stroke:
        raise ValueError("invalid travel")
    return c.spring_rate*(c.spring_free-(13-travel))

def parts(c=None,angle=0.,travel=0.):
    c=c or Core()
    if not math.isfinite(angle):
        raise ValueError("invalid angle")
    if travel<c.engagement+.4 and abs(angle/45-round(angle/45))>1e-8:
        raise ValueError("nonindexed pose requires released clutch")
    return {"stationary":stationary(c),"rotating":rotating(c).rotate((0,0,0),(0,0,1),angle),
            "plunger":plunger(c,travel)}

def effective_engagement(c,travel=0.):
    return max(0.,c.engagement-travel)
