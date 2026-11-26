import os
import pytest
import cadquery as cq
from itertools import combinations
from modular_arm.v4.flush_arm import AXES,LIMITS,blank,finished_flush,parts
from modular_arm.v4.structural_screen import measure

@pytest.mark.parametrize("index",[1,5])
def test_flush_blank(index):
    s=blank(index)
    lo,hi=LIMITS[index]
    assert s.isValid() and len(s.Solids())==1
    assert s.BoundingBox().zmin==pytest.approx(lo,abs=1e-5)
    assert s.BoundingBox().zmax==pytest.approx(hi,abs=1e-5)
    for x in (35,45):
        r=measure(finished_flush(index),x if index==5 else -x)
        assert r.centroid_z_mm==pytest.approx(AXES[index],abs=.02)
        assert min(r.wy_mm3,r.wz_mm3)>2000

@pytest.mark.parametrize("angle,travel",[(0,0),(45,0),(90,0),(135,0),(180,0),(30,7),(180,7)])
def test_cad_poses(angle,travel):
    p=parts(angle,travel)
    for n,s in p.items():
        assert s.isValid() and len(s.Solids())==1,n
    for (a,s),(b,t) in combinations(p.items(),2):
        assert s.intersect(t).Volume()<1e-6,(angle,travel,a,b)

@pytest.mark.parametrize("angle,travel",[(0,0),(90,0),(180,0),(30,7)])
def test_integrated_poses(angle,travel):
    from modular_arm.v4.integrated_arm import parts,audit
    archive=os.environ.get("ARM_REFERENCE_ZIP")
    if not archive:
        pytest.skip("set ARM_REFERENCE_ZIP")
    assert audit(parts(archive,angle,travel,flush=True,clocking=180.))==[]

def test_legacy_fold_interference_detected():
    from modular_arm.v4.integrated_arm import parts,audit
    archive=os.environ.get("ARM_REFERENCE_ZIP")
    if not archive:
        pytest.skip("set ARM_REFERENCE_ZIP")
    defects=audit(parts(archive,180,0,flush=True,clocking=0.))
    assert len(defects)==1
    a,b,volume=defects[0]
    assert (a,b)==("arm_a_integrated","arm_b_integrated")
    assert volume>0.01
