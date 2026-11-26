from itertools import combinations
import pytest
from modular_arm.assembly.modular_chain import modular_chain_parts,link_parts,PORT
from modular_arm.assembly.common_selector import CommonSelectorConfig

@pytest.mark.parametrize("spacing",[200.,240.])
def test_interchangeable_link(spacing):
    p=link_parts(CommonSelectorConfig(),spacing)["segment"]
    assert p.isValid() and len(p.Solids())==1
    assert p.BoundingBox().ylen==pytest.approx(spacing-112)

@pytest.mark.parametrize("angles,spacings",[((0.,),( )),((0.,0.),(200.,)),((-15.,15.),(240.,)),((0.,0.,0.),(240.,240.))])
def test_chain_geometry(angles,spacings):
    parts=modular_chain_parts(angles=angles,spacings=spacings)
    for name,p in parts.items():
        assert p.isValid() and len(p.Solids())==1,name
    for (a,p),(b,q) in combinations(parts.items(),2):
        x,y=p.BoundingBox(),q.BoundingBox()
        if x.xmax<y.xmin or y.xmax<x.xmin or x.ymax<y.ymin or y.ymax<x.ymin or x.zmax<y.zmin or y.zmax<x.zmin:
            continue
        assert p.intersect(q).Volume()<1e-6,(angles,a,b)
