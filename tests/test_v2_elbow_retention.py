import pytest
import cadquery as cq
from modular_arm.v2.elbow import Elbow,elbow_parts

def test_axle_positive_axial_capture():
    p=elbow_parts(angle=0)
    fixed,axle,keeper=p["fixed"],p["axle"],p["axle_keeper"]
    assert axle.intersect(keeper).Volume()<1e-6
    assert keeper.intersect(fixed).Volume()<1e-6
    assert axle.translate(cq.Vector(0,0,-1)).intersect(fixed).Volume()>1e-3
    assert keeper.translate(cq.Vector(0,0,1)).intersect(fixed).Volume()>1e-3
    assert axle.translate(cq.Vector(0,0,1)).intersect(keeper).Volume()>1e-3

@pytest.mark.parametrize("angle",range(-90,91,10))
def test_unlocked_elbow_sweep(angle):
    p=elbow_parts(angle=angle,locked=False)
    assert p["moving"].intersect(p["fixed"]).Volume()<1e-6
    assert p["moving"].intersect(p["axle"]).Volume()<1e-6
    assert p["moving"].intersect(p["axle_keeper"]).Volume()<1e-6
