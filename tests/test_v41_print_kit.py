import pytest
from modular_arm.v4.print_kit import quantities
@pytest.mark.parametrize("n",[2,3,4])
def test_quantities(n):
    q=quantities(n)
    assert q["tube_inner_standard"]+q["tube_inner_reclocked"]==n
    assert q["tube_outer"]==q["extension_screw"]==n
    assert q["head_a"]==q["head_b"]==q["button"]==n-1
