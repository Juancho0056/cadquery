import pytest
from modular_arm.assembly.common_selector import CommonSelectorConfig, return_components

@pytest.mark.parametrize("reverse",[False,True])
def test_return_components_valid(reverse):
    c=CommonSelectorConfig()
    for q in (0.,5.,10.,15.,20.):
        leaf,post,socket=return_components(c,reverse,q)
        for part in (leaf,post,socket):
            assert part.isValid() and len(part.Solids())==1
        assert leaf.intersect(socket).Volume()<1e-6
        assert leaf.intersect(post).Volume()<1e-6
        assert leaf.distance(post)<1e-5
