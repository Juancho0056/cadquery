import math
import pytest
import cadquery as cq
from modular_arm.mechanisms.dual_track import DualTrackConfig,profile_points,make_dual_track_rotor,make_follower,required_retraction

@pytest.fixture(scope="module")
def c():
    return DualTrackConfig()

@pytest.fixture(scope="module")
def rotor(c):
    return make_dual_track_rotor(c)

def test_rotor_single_valid_solid(c,rotor):
    assert rotor.isValid() and len(rotor.Solids())==1
    assert rotor.BoundingBox().zlen==pytest.approx(c.ring.thickness_mm,abs=1e-5)
    assert rotor.Volume()>0

def test_mirrored_profiles(c):
    a=profile_points(c)
    b=profile_points(c,True)
    assert len(a)==len(b)==4*c.ring.tooth_count
    for (x,y),(u,v) in zip(a,reversed(b)):
        assert x==pytest.approx(u) and y==pytest.approx(-v)

def test_bore_passes(c,rotor):
    p=cq.Solid.makeCylinder(c.ring.bore_diameter_mm/2-.1,c.ring.thickness_mm+2,cq.Vector(0,0,-1))
    assert rotor.intersect(p).Volume()<1e-7

def test_follower_tracks_are_axially_separate(c):
    p=make_follower(c)
    q=make_follower(c,reverse=True)
    assert p.isValid() and q.isValid()
    assert p.intersect(q).Volume()<1e-7

@pytest.mark.parametrize("reverse",[False,True])
def test_retracted_follower_clears_sampled_pitch(c,rotor,reverse):
    offset=c.tip_radius_mm-c.root_radius_mm+c.follower_width_mm
    p=make_follower(c,reverse=reverse,radial_offset_mm=offset)
    for i in range(13):
        assert rotor.rotate((0,0,0),(0,0,1),i*c.ring.pitch_degrees/12).intersect(p).Volume()<1e-7

@pytest.mark.parametrize("reverse",[False,True])
def test_sampled_retraction_exists(c,rotor,reverse):
    for angle in (0,c.ring.pitch_degrees/3,2*c.ring.pitch_degrees/3):
        d=required_retraction(c,rotor,angle,reverse)
        p=make_follower(c,reverse=reverse,radial_offset_mm=d)
        assert rotor.rotate((0,0,0),(0,0,1),angle).intersect(p).Volume()<1e-7
