import pytest
import cadquery as cq
from modular_arm.v4.straight_arm import AXES,finished_straight
from modular_arm.v4 import joint
from modular_arm.v4.structural_screen import measure

@pytest.mark.parametrize("index",[1,5])
def test_straight_section_and_core(index):
    s=finished_straight(index)
    old=joint.fixed() if index==1 else joint.moving()
    core=cq.Solid.makeCylinder(23.5,100,cq.Vector(0,0,-40))
    assert old.intersect(core).cut(s).Volume()<1e-6
    assert s.intersect(core).cut(old).Volume()<1e-6
    for x in (29,35,45):
        r=measure(s,x if index==5 else -x)
        assert r.centroid_z_mm==pytest.approx(AXES[index],abs=.02)
        assert r.wy_mm3>2000
        assert r.wz_mm3>2000

def test_nominal_tube_spacing():
    assert AXES[5]-AXES[1]==29.

@pytest.mark.parametrize("angle,travel",[(0,0),(45,0),(90,0),(30,7)])
def test_candidate_integrated_poses(angle,travel):
    import os
    from modular_arm.v4.integrated_arm import parts,audit
    archive=os.environ.get("ARM_REFERENCE_ZIP")
    if not archive:
        pytest.skip("set ARM_REFERENCE_ZIP")
    assert audit(parts(archive,angle,travel,straight=True))==[]
