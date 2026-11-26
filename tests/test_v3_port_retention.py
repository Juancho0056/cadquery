import pytest
import cadquery as cq
from modular_arm.v2.connector import Port,plug
from modular_arm.v3.modular_elbow import (
    compact_pin,retained_socket,retainer_screw,groove_z,
    GROOVE_WIDTH,GROOVE_RADIUS,RETAINER_LENGTH,RETAINER_TIP_Y,parts
)

@pytest.mark.parametrize("moving",[False,True])
def test_pin_capture(moving):
    c=Port()
    p=compact_pin(c,moving)
    s=retained_socket(c,moving)
    r=retainer_screw(c,moving)
    assert all(q.isValid() and len(q.Solids())==1 for q in (p,s,r))
    assert p.intersect(r).Volume()<1e-6
    assert r.intersect(s).Volume()<1e-6
    assert r.intersect(plug(c)).Volume()<1e-6
    # Both axial directions blocked by the screw or the pin head.
    for dz in (-3.,3.):
        q=p.translate(cq.Vector(0,0,dz))
        assert q.intersect(r).Volume()+q.intersect(s).Volume()>1e-3
    # Screw removed: axial insertion from head side remains possible.
    for d in range(0,41,2):
        q=p.translate(cq.Vector(0,0,d if moving else -d))
        assert q.intersect(s).Volume()<1e-6

def test_groove_outside_plug_shear_plane():
    c=Port()
    assert groove_z(c)-GROOVE_WIDTH/2>(c.height+c.clearance)/2
    assert groove_z(c)+GROOVE_WIDTH/2<c.outer_height/2
    assert 0<GROOVE_RADIUS<c.pin/2
    assert RETAINER_TIP_Y+RETAINER_LENGTH<c.outer_width/2

@pytest.mark.parametrize("angle",[-180,-165,0,165,180])
def test_retainer_folded(angle):
    from itertools import combinations
    p=parts(angle,7.,connected=True)
    for (a,s),(b,t) in combinations(p.items(),2):
        assert s.intersect(t).Volume()<1e-6,(angle,a,b)
