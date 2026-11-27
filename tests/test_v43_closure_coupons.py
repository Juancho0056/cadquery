import pytest
from modular_arm.v43 import closure_coupons as c
@pytest.mark.parametrize("name",["thread","bayonet"])
def test_closure(name):
    base,cap=(c.threaded if name=="thread" else c.bayonet)()
    assert all(x["overlap_mm3"]<=.01 for x in c.path_samples(name,base,cap))
    if name=="bayonet": cap=c.turn(cap,40.)
    base,cap,key=c.lock_pair(base,cap)
    assert base.intersect(cap).Volume()<=.01
    assert key.intersect(base).Volume()+key.intersect(cap).Volume()<=.01
    assert all(key.intersect(c.turn(cap,a)).Volume()>.01 for a in (-5.,5.))
    assert base.intersect(cap.translate(c.V(0,0,2.))).Volume()>.01
