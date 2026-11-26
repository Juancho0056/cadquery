from itertools import combinations
import pytest
from modular_arm.assembly.serial_arm import serial_arm_parts

@pytest.mark.parametrize("a,b",[(0.,0.),(-30.,30.),(30.,-30.)])
def test_serial_arm_indexed_poses(a,b):
    parts=serial_arm_parts(first_angle=a,second_angle=b)
    for name,p in parts.items():
        assert p.isValid() and len(p.Solids())==1,name
    for (name,p),(other,q) in combinations(parts.items(),2):
        # Bounding-box rejection avoids needless kernel operations.
        x,y=p.BoundingBox(),q.BoundingBox()
        if x.xmax<y.xmin or y.xmax<x.xmin or x.ymax<y.ymin or y.ymax<x.ymin or x.zmax<y.zmin or y.zmax<x.zmin:
            continue
        assert p.intersect(q).Volume()<1e-6,(a,b,name,other)

def test_serial_limits():
    with pytest.raises(ValueError):
        serial_arm_parts(first_angle=45.)
