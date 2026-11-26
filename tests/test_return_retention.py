import cadquery as cq
import pytest
from modular_arm.assembly.common_selector import CommonSelectorConfig,return_components,return_retainer_parts

@pytest.mark.parametrize("reverse",[False,True])
def test_return_contact_sweep(reverse):
    c=CommonSelectorConfig()
    for q in range(21):
        leaf,post,socket=return_components(c,reverse,q)
        assert leaf.intersect(post).Volume()<1e-6,q
        assert leaf.distance(post)<1e-5,q
        assert leaf.intersect(socket).Volume()<1e-6,q

@pytest.mark.parametrize("reverse",[False,True])
def test_root_positive_vertical_retention(reverse):
    c=CommonSelectorConfig()
    leaf,post,socket=return_components(c,reverse,0.)
    parts=return_retainer_parts(c,reverse)
    pin=parts["grooved_pin"]
    clip=parts["snap_keeper"]
    for p in parts.values():
        assert p.isValid() and len(p.Solids())==1
        assert p.intersect(socket).Volume()<1e-6
        assert p.intersect(leaf).Volume()<1e-6
    assert pin.intersect(clip).Volume()<1e-6
    assert leaf.translate(cq.Vector(0,0,1.)).intersect(pin).Volume()>1e-3
    assert leaf.translate(cq.Vector(0,1.,0)).intersect(socket).Volume()>1e-3
