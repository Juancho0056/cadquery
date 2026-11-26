import pytest
import cadquery as cq
from modular_arm.v2.elbow import Elbow,elbow_parts
from modular_arm.v2.connector import plug,locking_pin

@pytest.mark.parametrize("angle",[-90,0,60,90])
def test_elbow_ports_accept_standard_plugs(angle):
    c=Elbow()
    parts=elbow_parts(c,angle)
    left=plug(c.port).rotate((0,0,0),(0,0,1),180).translate(cq.Vector(-c.mouth,0,0))
    right=plug(c.port).translate(cq.Vector(c.mouth,0,0)).rotate((0,0,0),(0,0,1),angle)
    for p in (left,right):
        for name,q in parts.items():
            assert p.intersect(q).Volume()<1e-6,(angle,name)

def test_invalid_socket_offset():
    with pytest.raises(ValueError):
        Elbow(mouth=65.)
