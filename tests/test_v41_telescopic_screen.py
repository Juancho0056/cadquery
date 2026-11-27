import os
import numpy as np
import pytest
from modular_arm.v4.telescopic_screen import section,report

def box_triangles(r):
    # Closed square prism; only lateral faces intersect the test plane.
    p=np.array([[-r,-r],[r,-r],[r,r],[-r,r]])
    triangles=[]
    for j in range(4):
        a,b=p[j],p[(j+1)%4]
        v=np.array([[*a,0],[*b,0],[*b,2],[*a,2]])
        triangles.extend([v[[0,1,2]],v[[0,2,3]]])
    return np.array(triangles)

def test_solid_and_hollow():
    outer=box_triangles(10)
    solid=section(outer,(0,0),.7)
    hollow=section(np.concatenate([outer,box_triangles(6)]),(0,0),.7)
    assert solid["axis_occupied"]
    assert not hollow["axis_occupied"]
    assert hollow["nearest_boundary_mm"]==pytest.approx(6)
    assert solid["nearest_boundary_mm"]==pytest.approx(10)

def test_original_reference():
    archive=os.environ.get("ARM_REFERENCE_ZIP")
    if not archive:
        pytest.skip("set ARM_REFERENCE_ZIP")
    r=report(archive)
    assert all(s["axis_occupied"] for s in r["parts"]["1"])
    assert r["candidate_b_radial_margin_mm"]==pytest.approx(1.8,abs=.05)
    assert r["status"]=="sampled_reference_sections_only"
