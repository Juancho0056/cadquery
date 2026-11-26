from itertools import combinations
import pytest
import cadquery as cq
from modular_arm.assembly.integrated_indexing import IntegratedConfig, integrated_parts
from modular_arm.mechanisms.selector_contract import Mode,requested_direction_allowed

@pytest.fixture(scope="module")
def locked():
    return integrated_parts()

def test_integrated_solids(locked):
    for name,p in locked.items():
        assert p.isValid() and len(p.Solids())==1,name

def test_static_interferences(locked):
    for (a,p),(b,q) in combinations(locked.items(),2):
        assert p.intersect(q).Volume()<1e-6,(a,b)

def test_independent_lock_blocks_rotor(locked):
    p=locked["rotor"]
    q=locked["angular_lock"]
    for angle in (-5,5):
        assert p.rotate((0,0,0),(0,0,1),angle).intersect(q).Volume()>1e-3

def test_lock_insertion_path(locked):
    pin=locked["angular_lock"]
    c=IntegratedConfig()
    for offset in (0,5,15,30,c.joint.stack_height_mm+1):
        shifted=pin.translate(cq.Vector(0,0,offset))
        for name,part in locked.items():
            if name!="angular_lock":
                assert shifted.intersect(part).Volume()<1e-6,(name,offset)

@pytest.mark.parametrize("mode,clockwise,lock,expected",[
    (Mode.A,True,False,True),(Mode.A,False,False,False),
    (Mode.B,True,False,False),(Mode.B,False,False,False),
    (Mode.C,True,False,False),(Mode.C,False,False,True),
    (Mode.A,True,True,False),(Mode.C,False,True,False)])
def test_selector_contract(mode,clockwise,lock,expected):
    assert requested_direction_allowed(mode,clockwise,lock)==expected
