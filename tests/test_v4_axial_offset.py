import pytest
import cadquery as cq
from modular_arm.v4.transition import finished_half
from modular_arm.v4.integrated_arm import axis_heights
from modular_arm.v4 import joint

@pytest.mark.parametrize("offset",[0.,5.,10.,15.4])
def test_axis_heights(offset):
    z=axis_heights(offset)
    assert z[5]-z[1]==pytest.approx(offset)
    assert (z[5]+z[1])/2==pytest.approx(11.7)

@pytest.mark.parametrize("offset",[-1.,16.,float("nan")])
def test_invalid_offset(offset):
    with pytest.raises(ValueError):
        axis_heights(offset)

@pytest.mark.parametrize("index",[1,5])
@pytest.mark.parametrize("offset",[0.,10.])
def test_variant_core_and_terminal(index,offset):
    z=axis_heights(offset)[index]
    s=finished_half(index,axis_z=z)
    ref=joint.fixed() if index==1 else joint.moving()
    core=cq.Solid.makeCylinder(28,60,cq.Vector(0,0,-15))
    assert s.isValid() and len(s.Solids())==1
    assert ref.intersect(core).cut(s).Volume()<1e-6
    assert s.intersect(core).cut(ref).Volume()<1e-6
    x=(-1 if index==1 else 1)*46
    bb=s.intersect(cq.Solid.makeBox(.02,100,100,cq.Vector(x-.01,-50,-50))).BoundingBox()
    assert (bb.zmin+bb.zmax)/2==pytest.approx(z,abs=.15)

def test_default_axis_heights_exact():
    from modular_arm.v4.integrated_arm import CENTER_Z
    assert axis_heights(15.4)==CENTER_Z

def test_aligned_longitudinal_envelopes_overlap_when_folded():
    # Conservative envelope control, not a claim about the CAD necks.
    z=axis_heights(0.)
    a=cq.Solid.makeCylinder(13.95,30,cq.Vector(-46,0,z[1]),cq.Vector(-1,0,0))
    b=cq.Solid.makeCylinder(13.95,30,cq.Vector(46,0,z[5]),cq.Vector(1,0,0))
    assert a.intersect(b).Volume()<1e-6
    assert a.intersect(joint.turn(b,180)).Volume()>1.

def test_fold_intersection_detector_positive_control():
    a=cq.Solid.makeBox(10,10,10)
    assert a.intersect(a.translate(cq.Vector(5,0,0))).Volume()==pytest.approx(500.)
