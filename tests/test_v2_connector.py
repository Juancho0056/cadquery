import pytest
import cadquery as cq
from itertools import combinations
from modular_arm.v2.connector import Port,socket,plug,locking_pin,segment,assembly

@pytest.mark.parametrize("length",[60.,100.,150.,200.])
def test_interchangeable_segments(length):
    p=assembly(length)
    for name,s in p.items():
        assert s.isValid() and len(s.Solids())==1,name
    for (a,s),(b,t) in combinations(p.items(),2):
        assert s.intersect(t).Volume()<1e-6,(a,b)

def test_unlocked_insertion():
    c=Port()
    for x in range(0,26):
        assert socket(c).intersect(plug(c).translate(cq.Vector(x,0,0))).Volume()<1e-6,x

def test_pin_positive_retention():
    c=Port()
    p=plug(c)
    pin=locking_pin(c)
    assert p.intersect(pin).Volume()<1e-6
    assert p.translate(cq.Vector(1,0,0)).intersect(pin).Volume()>1e-3

def test_socket_blocks_transverse_motion():
    c=Port()
    for displacement in ((0,1,0),(0,-1,0),(0,0,1),(0,0,-1)):
        assert socket(c).intersect(plug(c).translate(cq.Vector(*displacement))).Volume()>1e-3

def test_invalid_port():
    with pytest.raises(ValueError):
        Port(pin_center=1.)
