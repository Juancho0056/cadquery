"""CAD-only tests; no original ZIP or mesh backend required."""
from itertools import combinations
import pytest
import cadquery as cq
from modular_arm.v4 import joint
from modular_arm.v4.transition import outer_blank,finished_half

@pytest.fixture(scope="module")
def halves():
    return {i:finished_half(i) for i in (1,5)}

@pytest.mark.parametrize("index",[1,5])
def test_blank_exact_core(index):
    s=outer_blank(index)
    assert s.isValid() and len(s.Solids())==1
    base=cq.Solid.makeCylinder(28,22 if index==1 else 14,
                              cq.Vector(0,0,-10 if index==1 else 12.4))
    core=cq.Solid.makeCylinder(28,100,cq.Vector(0,0,-40))
    assert base.cut(s).Volume()<1e-6
    assert s.intersect(core).cut(base).Volume()<1e-6

@pytest.mark.parametrize("index",[1,5])
def test_finished_core_unchanged(halves,index):
    ref=joint.fixed() if index==1 else joint.moving()
    s=halves[index]
    core=cq.Solid.makeCylinder(28,60,cq.Vector(0,0,-15))
    assert s.isValid() and len(s.Solids())==1
    assert ref.intersect(core).cut(s).Volume()<1e-6
    assert s.intersect(core).cut(ref).Volume()<1e-6

@pytest.mark.parametrize("index",[1,5])
def test_terminal_section(halves,index):
    sign=-1 if index==1 else 1
    for x in (46,47.9):
        cut=cq.Solid.makeBox(.02,100,100,cq.Vector(sign*x-.01,-50,-50))
        section=halves[index].intersect(cut)
        assert len(section.Solids())==1
        bb=section.BoundingBox()
        assert 27.7<=bb.ylen<=28.1
        assert 27.7<=bb.zlen<=28.1
        assert 600<=section.Volume()/.02<=625

@pytest.mark.parametrize("angle,travel",[(0,0),(45,0),(-45,0),(90,0),(-90,0),(30,7)])
def test_transition_operating_poses(halves,angle,travel):
    p={"a":halves[1],"b":joint.turn(halves[5],angle),
       "button":joint.button(travel),**joint.screws(angle)}
    for (a,s),(b,t) in combinations(p.items(),2):
        assert s.intersect(t).Volume()<1e-6,(angle,travel,a,b)

@pytest.mark.parametrize("angle",[135,180])
def test_known_fold_collision_is_detected(halves,angle):
    # This is explicitly a negative control, NOT an approved operating pose.
    assert halves[1].intersect(joint.turn(halves[5],angle)).Volume()>1.
