import pytest
from modular_arm.v4 import alternating_chain as a
from modular_arm.v4 import chain_motion_screen as s

def test_nonindexed_frame():
    f=a.frames((15.,0.),(6,6,6))
    assert len(f)==2
    assert f[1][2,2]==pytest.approx(-1.)

def test_invalid_angle():
    with pytest.raises(ValueError): a.frames((181.,0.),(6,6,6))

def test_pose_contract():
    with pytest.raises(ValueError): a.validate_pose(15.,0.)
    with pytest.raises(ValueError): a.validate_pose(0.,8.)
    a.validate_pose(15.,7.)

@pytest.mark.parametrize("name,expected",[
    ("joint_1_head_a",0),("joint_1_button",0),("joint_1_a_screw",0),
    ("joint_1_head_b",1),("joint_1_b_nut",1),("joint_1_retainer_2",1),
    ("segment_1_inner",0),("segment_2_inner",1),("segment_2_lock",1),
    ("joint_0_head_b",0),("joint_2_head_a",1)])
def test_group(name,expected):
    assert s.group(name,1)==expected

def test_cache_preserves_invariant_defects(monkeypatch):
    parts={"joint_0_head_a":object(),"joint_0_button":object(),"joint_0_head_b":object()}
    calls=[]
    def measure(a,b):
        calls.append((a,b))
        return 2.
    monkeypatch.setattr(s,"overlap",measure)
    audit=s.RigidAudit(parts,0)
    assert len(calls)==1
    assert len(audit.audit(parts))==3
    assert len(audit.audit(parts))==3
    assert len(calls)==5

def test_atomic_checkpoint(tmp_path):
    path=tmp_path/"report.json"
    s.save(path,{"complete":False})
    assert path.exists()
    assert not path.with_suffix(".tmp").exists()

def test_campaign_size():
    assert len(s.jobs("full"))==22
    assert len(s.jobs("quick"))==4

def test_equivalence_reference():
    import os
    archive=os.environ.get("ARM_REFERENCE_ZIP")
    if not archive: pytest.skip("set ARM_REFERENCE_ZIP")
    source=a.templates(archive)
    cache=a.JointMeshes()
    def pose(angle):
        return a.assemble(archive,(0.,angle),(6,6,6),source=source,travels=(0.,7.),joint_meshes=cache)[0]
    audit=s.RigidAudit(pose(0.),1)
    for angle in (0.,90.,165.,180.):
        parts=pose(angle)
        expected={(x,y):v for x,y,v in a.ia.audit(parts)}
        actual={(x,y):v for x,y,v in audit.audit(parts)}
        assert actual.keys()==expected.keys()
        assert actual==pytest.approx(expected,abs=.01)
