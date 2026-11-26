import numpy as np
import pytest
from modular_arm.v4.printability import overhang_report

def test_downward_face_detected():
    t=np.array([[[0,0,0],[1,0,0],[0,1,0]],[[0,0,2],[0,1,2],[1,0,2]]])
    r=overhang_report(t,(0,0,1))
    assert r["candidate_facets"]==1
    assert r["unsupported_candidate_area_mm2"]==pytest.approx(.5)

def test_vertical_face_not_overhang():
    t=np.array([[[0,0,0],[0,1,0],[0,1,2]]])
    assert overhang_report(t,(0,0,1))["candidate_facets"]==0

@pytest.mark.parametrize("i",[1,5])
def test_ramp_limits_and_terminal(i):
    from modular_arm.v4.flush_arm import blank,LIMITS,AXES
    from modular_arm.v4.structural_screen import measure
    s=blank(i)
    assert s.isValid() and len(s.Solids())==1
    assert s.BoundingBox().zmin==pytest.approx(LIMITS[i][0],abs=1e-5)
    assert s.BoundingBox().zmax==pytest.approx(LIMITS[i][1],abs=1e-5)
    r=measure(s,45.5 if i==5 else -45.5)
    assert r.centroid_z_mm==pytest.approx(AXES[i],abs=.02)

@pytest.mark.parametrize("direction",[-1,1])
def test_regional_partition(direction):
    from modular_arm.v4.printability import regional_overhang
    t=np.array([[[x*direction,0,0],[x*direction,1,0],[x*direction,0,1]] for x in (10,35,60)])
    r=regional_overhang(t,[0,1,2],direction=direction)
    assert all(v["facets"]==1 and v["area_mm2"]==pytest.approx(.5) for v in r.values())

def test_empty_regional_partition():
    from modular_arm.v4.printability import regional_overhang
    r=regional_overhang(np.zeros((1,3,3)),[],direction=1)
    assert all(v["facets"]==0 and v["area_mm2"]==0 for v in r.values())
