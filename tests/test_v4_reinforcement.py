import pytest
import cadquery as cq
from modular_arm.v4.transition import finished_half
from modular_arm.v4.structural_screen import measure
from modular_arm.v4.integrated_arm import axis_heights

@pytest.mark.parametrize("lift",[4.,6.])
def test_functional_core_and_section_improvement(lift):
    old=finished_half(5)
    new=finished_half(5,reinforcement=lift)
    assert new.isValid() and len(new.Solids())==1
    # All functional features within radius 23.5 remain identical.
    core=cq.Solid.makeCylinder(23.5,60,cq.Vector(0,0,-15))
    assert old.intersect(core).cut(new).Volume()<1e-6
    assert new.intersect(core).cut(old).Volume()<1e-6
    # Strength proxy at the measured critical station; not a load rating.
    assert measure(new,28).wy_mm3>measure(old,28).wy_mm3*1.2

@pytest.mark.parametrize("lift",[4.,6.])
def test_terminal_geometry(lift):
    old=finished_half(5)
    new=finished_half(5,reinforcement=lift)
    for x in (46.,47.9):
        slab=cq.Solid.makeBox(.02,100,100,cq.Vector(x-.01,-50,-50))
        a,b=old.intersect(slab),new.intersect(slab)
        assert abs(a.Volume()-b.Volume())<.02
        assert abs(a.BoundingBox().zmin-b.BoundingBox().zmin)<.1
        assert abs(a.BoundingBox().zmax-b.BoundingBox().zmax)<.1

@pytest.mark.parametrize("lift",[-1.,7.,float("nan")])
def test_invalid_reinforcement(lift):
    with pytest.raises(ValueError):
        finished_half(5,reinforcement=lift)

def test_no_reinforcement_a():
    with pytest.raises(ValueError):
        finished_half(1,reinforcement=4.)

@pytest.mark.parametrize("lift",[4.,6.])
def test_integrated_candidate(lift):
    import os
    from modular_arm.v4.integrated_arm import parts,audit
    archive=os.environ.get("ARM_REFERENCE_ZIP")
    if not archive:
        pytest.skip("set ARM_REFERENCE_ZIP")
    for angle,travel in ((0,0),(45,0),(90,0),(30,7)):
        assert audit(parts(archive,angle,travel,reinforcement=lift))==[]
