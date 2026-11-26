"""Experimental two-track sawtooth rotor and radial follower bench.
No automatic ratchet or load capacity is asserted.
"""
from dataclasses import dataclass, field
import math
import cadquery as cq
from modular_arm.config.ring import ExperimentalRingConfig

@dataclass(frozen=True, slots=True)
class DualTrackConfig:
    ring: ExperimentalRingConfig = field(default_factory=ExperimentalRingConfig)
    track_axial_gap_mm: float = 2.0
    follower_width_mm: float = 2.0
    follower_length_mm: float = 12.0
    follower_clearance_mm: float = .4
    crest_fraction: float = .08
    root_fraction: float = .12

    def __post_init__(self):
        vals=(self.track_axial_gap_mm,self.follower_width_mm,self.follower_length_mm,
              self.follower_clearance_mm,self.crest_fraction,self.root_fraction)
        if not all(math.isfinite(v) and v>0 for v in vals):
            raise ValueError("positive finite dimensions required")
        if self.track_axial_gap_mm>=self.ring.thickness_mm:
            raise ValueError("gap must fit inside rotor")
        if self.crest_fraction+self.root_fraction>=.5:
            raise ValueError("invalid tooth proportions")
        if self.follower_clearance_mm>=self.track_thickness_mm:
            raise ValueError("follower axial clearance too large")
        if self.follower_width_mm>=2*self.root_radius_mm*math.sin(math.pi/self.ring.tooth_count):
            raise ValueError("follower exceeds pitch width")

    @property
    def track_thickness_mm(self):
        return (self.ring.thickness_mm-self.track_axial_gap_mm)/2

    @property
    def root_radius_mm(self):
        return self.ring.root_diameter_mm/2

    @property
    def tip_radius_mm(self):
        return self.ring.outer_diameter_mm/2

    @property
    def follower_thickness_mm(self):
        return self.track_thickness_mm-self.follower_clearance_mm


def profile_points(c, reverse=False):
    """Polygon in XY; mirrored winding produces opposite slopes."""
    points=[]
    pitch=2*math.pi/c.ring.tooth_count
    for i in range(c.ring.tooth_count):
        for fraction,r in ((0,c.root_radius_mm),(c.root_fraction,c.root_radius_mm),
                           (1-c.crest_fraction,c.tip_radius_mm),(1-c.crest_fraction/2,c.tip_radius_mm)):
            a=(i+fraction)*pitch
            if reverse:
                a=-a
            points.append((r*math.cos(a),r*math.sin(a)))
    return list(reversed(points)) if reverse else points


def make_dual_track_rotor(c=None):
    c=c or DualTrackConfig()
    t=c.track_thickness_mm
    low=cq.Workplane("XY").polyline(profile_points(c)).close().extrude(t)
    core=cq.Workplane("XY").circle(c.root_radius_mm).extrude(c.ring.thickness_mm)
    high=cq.Workplane("XY").polyline(profile_points(c,True)).close().extrude(t).translate((0,0,t+c.track_axial_gap_mm))
    bore=cq.Workplane("XY").circle(c.ring.bore_diameter_mm/2).extrude(c.ring.thickness_mm)
    return core.union(low).union(high).cut(bore).val()


def make_follower(c=None, *, reverse=False, radial_offset_mm=0.):
    """Radially translated coupon; not the final pivoting pawl."""
    c=c or DualTrackConfig()
    if not math.isfinite(radial_offset_mm) or radial_offset_mm<0:
        raise ValueError("invalid follower displacement")
    z=c.follower_clearance_mm/2+(c.track_thickness_mm+c.track_axial_gap_mm if reverse else 0)
    # Tip points toward the rotor axis. Position clear of the root.
    x=c.root_radius_mm+c.follower_clearance_mm+radial_offset_mm
    w=c.follower_width_mm/2
    l=c.follower_length_mm
    return cq.Workplane("XY").polyline([(x,0),(x+w,-w),(x+l,-w),(x+l,w),(x+w,w)]).close().extrude(c.follower_thickness_mm).val().translate(cq.Vector(0,0,z))


def bench_parts(c=None, *, angle_deg=0., upper_offset_mm=0., lower_offset_mm=0.):
    c=c or DualTrackConfig()
    return {"rotor":make_dual_track_rotor(c).rotate((0,0,0),(0,0,1),angle_deg),
            "lower_follower":make_follower(c,radial_offset_mm=lower_offset_mm),
            "upper_follower":make_follower(c,reverse=True,radial_offset_mm=upper_offset_mm)}


def required_retraction(c, rotor, angle_deg, reverse=False, step_mm=.25):
    """Sample a collision-free radial position, not force-driven motion."""
    if not math.isfinite(step_mm) or step_mm<=0:
        raise ValueError("positive finite sampling step required")
    posed=rotor.rotate((0,0,0),(0,0,1),angle_deg)
    travel=c.tip_radius_mm-c.root_radius_mm+c.follower_width_mm
    for i in range(math.ceil(travel/step_mm)+1):
        offset=i*step_mm
        if posed.intersect(make_follower(c,reverse=reverse,radial_offset_mm=offset)).Volume()<1e-7:
            return offset
    raise RuntimeError("no sampled collision-free position")
