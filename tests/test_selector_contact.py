import math
import pytest
from modular_arm.assembly.common_selector import CommonSelectorConfig,lever,slider

@pytest.mark.parametrize("reverse",[False,True])
def test_full_sampled_contact_path(reverse):
    c=CommonSelectorConfig()
    sign=-1 if reverse else 1
    for i in range(21):
        q=c.contact.full_release_deg*i/20
        d=sign*(c.neutral_gap_mm+c.lever_radius_mm*math.sin(math.radians(q)))
        p=lever(c,reverse,q)
        s=slider(c,d)
        assert p.intersect(s).Volume()<1e-6,(q,reverse)
        assert p.distance(s)<1e-5,(q,reverse,"lost contact")
        inactive=lever(c,not reverse,0.)
        assert inactive.intersect(s).Volume()<1e-6,(q,reverse,"inactive interference")

@pytest.mark.parametrize("reverse",[False,True])
def test_neutral_leaves_active_lever_free(reverse):
    c=CommonSelectorConfig()
    s=slider(c,0.)
    for i in range(21):
        p=lever(c,reverse,c.contact.full_release_deg*i/20)
        assert p.intersect(s).Volume()<1e-6,(i,reverse)
