from itertools import combinations
import cadquery as cq
from modular_arm.mechanisms.retention import keeper_parts,KeeperConfig
from modular_arm.config.joint import JointConfig
from modular_arm.assembly.joint_keeper import joint_with_keeper

def test_keeper_geometry_and_fit():
    p,q=keeper_parts().values()
    assert p.isValid() and q.isValid()
    assert len(p.Solids())==len(q.Solids())==1
    assert p.intersect(q).Volume()<1e-6

def test_axial_capture():
    p,q=keeper_parts().values()
    for z in (-1,1):
        assert p.translate(cq.Vector(0,0,z)).intersect(q).Volume()>1e-3

def test_rigid_radial_installation_obstructed():
    p,q=keeper_parts().values()
    assert p.intersect(q.translate(cq.Vector(0,-2,0))).Volume()>1e-3

def test_joint_keeper_static_fit():
    j=JointConfig()
    assert KeeperConfig.from_joint(j)==KeeperConfig()
    parts=joint_with_keeper(j)
    for (a,p),(b,q) in combinations(parts.items(),2):
        assert p.intersect(q).Volume()<1e-6,(a,b)
