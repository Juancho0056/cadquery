import pytest
import cadquery as cq
from modular_arm.v4.transition import roof_height,roof_envelope,finished_half
from modular_arm.v4.structural_screen import measure

@pytest.mark.parametrize("z",[11.7,19.4])
@pytest.mark.parametrize("lift",[4.,6.])
def test_roof_bounds(z,lift):
    for a,b in ((24,32),(32,46)):
        lo,hi=sorted((roof_height(a,z,lift),roof_height(b,z,lift)))
        vals=[roof_height(a+(b-a)*i/100,z,lift) for i in range(101)]
        assert min(vals)>=lo-1e-9
        assert max(vals)<=hi+1e-9
        assert all((v-u)*(vals[-1]-vals[0])>=-1e-9 for u,v in zip(vals,vals[1:]))
    for x in (24,32,46):
        assert abs((roof_height(x+.001,z,lift)-roof_height(x-.001,z,lift))/.002)<.01

@pytest.mark.parametrize("z",[11.7,19.4])
@pytest.mark.parametrize("lift",[4.,6.])
def test_actual_roof_and_core(z,lift):
    s=finished_half(5,axis_z=z,reinforcement=lift)
    old=finished_half(5,axis_z=z)
    assert s.isValid() and len(s.Solids())==1
    roof=roof_envelope(z,lift)
    assert s.cut(roof).Volume()<1e-5
    core=cq.Solid.makeCylinder(23.5,60,cq.Vector(0,0,-15))
    assert s.intersect(core).cut(old).Volume()<1e-6
    assert old.intersect(core).cut(s).Volume()<1e-6
    rows=[measure(s,x) for x in range(28,46)]
    baseline=[measure(old,x) for x in range(28,46)]
    assert min(r.wy_mm3 for r in rows)>min(r.wy_mm3 for r in baseline)*1.2
