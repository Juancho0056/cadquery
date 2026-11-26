import numpy as np
import pytest
from modular_arm.v4.orientation_screen import compare

@pytest.mark.parametrize("direction",[-1,1])
def test_orientation_comparison(direction):
    t=np.array([[[0,0,0],[0,1,0],[0,0,1]],[[10,0,0],[10,0,1],[10,1,0]]])
    r=compare(t,direction)
    assert len(r)==17
    assert len({row["name"] for row in r})==17
    assert r[0]["build_axis"]==[-direction,0,0]
    for row in r:
        assert np.linalg.norm(row["build_axis"])==pytest.approx(1)
        assert sum(x["area_mm2"] for x in row["regions"].values())==pytest.approx(row["unsupported_candidate_area_mm2"])
