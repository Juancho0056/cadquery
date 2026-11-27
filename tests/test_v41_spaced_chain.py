import numpy as np
import pytest
from modular_arm.v4 import spaced_chain as s

def test_two_levels():
    f=s.frames((0.,0.,0.),(6,6,6,6))
    z=s.s.f.AXES
    heights=[(f[0]@s.offset(1)@np.array([0.,0.,z[1],1.]))[2]]
    heights.extend((t@s.offset(5)@np.array([0.,0.,z[5],1.]))[2] for t in f)
    assert heights==pytest.approx([-2.5,26.9,-2.5,26.9])

@pytest.mark.parametrize("angle",[0.,90.,180.])
def test_shared_outer_registration(angle):
    f=s.frames((angle,0.),(6,6,6))
    incoming=f[1]@s.offset(1)@s.s.clock.module_transform(180.)@np.linalg.inv(s.c.segment_transform(6))
    outgoing=f[0]@s.c.rotation(angle)@s.offset(5)
    assert incoming==pytest.approx(outgoing,abs=1e-9)

def test_reference_local_equivalence():
    import os
    archive=os.environ.get("ARM_REFERENCE_ZIP")
    if not archive: pytest.skip("set ARM_REFERENCE_ZIP")
    b=s.Builder(archive)
    for angle in (0.,180.):
        parts,_=b.parts((angle,),(12,12))
        assert not s.ia.audit(parts)
