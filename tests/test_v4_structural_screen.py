import math
import pytest
import cadquery as cq
import numpy as np
from modular_arm.v4.structural_screen import measure,polygon_section,screen

def test_rectangle_exact_integrals():
    t=np.array([[[-2,-3],[2,-3],[2,3]],[[-2,-3],[2,3],[-2,3]]])
    a,y,z,iy,iz,iyz,wy,wz=polygon_section(t)
    assert (a,y,z,iy,iz,iyz,wy,wz)==pytest.approx((24,0,0,72,32,0,24,16))
    assert polygon_section(t[:,::-1,:])==pytest.approx((a,y,z,iy,iz,iyz,wy,wz))

def test_rectangle_cad():
    s=cq.Solid.makeBox(10,4,6,cq.Vector(0,-2,-3))
    r=measure(s,5)
    assert (r.area_mm2,r.iy_mm4,r.iz_mm4,r.wy_mm3,r.wz_mm3)==pytest.approx((24,72,32,24,16),rel=1e-5)

def test_circle_cad():
    s=cq.Solid.makeCylinder(10,20,cq.Vector(0,0,0),cq.Vector(1,0,0))
    r=measure(s,10)
    assert r.area_mm2==pytest.approx(math.pi*100,rel=.015)
    assert r.iy_mm4==pytest.approx(math.pi*10**4/4,rel=.025)
    assert r.iz_mm4==pytest.approx(r.iy_mm4,rel=.025)

def test_hollow_section_preserves_void():
    a=cq.Solid.makeCylinder(10,20,cq.Vector(0,0,0),cq.Vector(1,0,0))
    b=cq.Solid.makeCylinder(5,22,cq.Vector(-1,0,0),cq.Vector(1,0,0))
    r=measure(a.cut(b),10)
    assert r.area_mm2==pytest.approx(math.pi*75,rel=.02)
    assert r.iy_mm4==pytest.approx(math.pi*(10**4-5**4)/4,rel=.03)

@pytest.mark.parametrize("offset",[0.,15.4])
def test_actual_neck_screen(offset):
    r=screen(offset,stations=(28,32,38,44,45))
    for h in r["halves"].values():
        assert len(h["sections"])==5
        assert h["minimum_area"]["area_mm2"]>0
        assert h["minimum_wy"]["wy_mm3"]>0
        assert h["minimum_wz"]["wz_mm3"]>0
