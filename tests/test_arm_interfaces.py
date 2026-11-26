from itertools import combinations
import pytest
import cadquery as cq
from modular_arm.assembly.common_selector import CommonSelectorConfig,common_selector_parts
from modular_arm.assembly.arm_interfaces import ArmInterfaceConfig,interface_solids

@pytest.fixture(scope="module")
def baseline():
    c=CommonSelectorConfig()
    return c,common_selector_parts(c)

def test_interfaces_join_structural_solids(baseline):
    c,p=baseline
    fixed,moving=interface_solids(c)
    for name,arm in (("housing",fixed),("rotor",moving)):
        assert arm.isValid() and len(arm.Solids())==1
        assert p[name].intersect(arm).Volume()>1e-3
        solid=p[name].fuse(arm)
        assert solid.isValid() and len(solid.Solids())==1

def test_mounting_bores(baseline):
    c,_=baseline
    a=ArmInterfaceConfig()
    fixed,moving=interface_solids(c,a)
    for d in a.mounting_centers_mm:
        for part,x,y in ((fixed,d,0),(moving,0,-d)):
            probe=cq.Solid.makeCylinder(a.mounting_hole_mm/2-.05,100,cq.Vector(x,y,0))
            assert part.intersect(probe).Volume()<1e-6

def test_arm_envelope_sampled_sweep(baseline):
    c,p=baseline
    a=ArmInterfaceConfig()
    fixed,moving=interface_solids(c,a)
    obstacles={name:shape for name,shape in p.items() if name!="rotor"}
    obstacles["fixed_arm"]=fixed
    for i in range(13):
        angle=-a.sweep_limit_deg+2*a.sweep_limit_deg*i/12
        arm=moving.rotate((0,0,0),(0,0,1),angle)
        for name,shape in obstacles.items():
            assert arm.intersect(shape).Volume()<1e-6,(angle,name)
