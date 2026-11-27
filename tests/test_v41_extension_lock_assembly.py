import pytest
from modular_arm.v4 import extension_lock_assembly as a

def test_restraint_detects_obstruction():
    tm=a.l.e.ia.mesh_library()
    inner=tm.creation.box(extents=(1.,2.,2.))
    screw=tm.creation.box(extents=(1.,2.,2.))
    screw.apply_translation((1.5,0.,0.))
    assert a.overlap(inner,screw)==0.
    rows=a.restraint(inner,screw)
    assert next(r for r in rows if r["shift_mm"]==1.)["screw_overlap_mm3"]==pytest.approx(2.)
    assert next(r for r in rows if r["shift_mm"]==-1.)["screw_overlap_mm3"]==0.

def test_index_range():
    assert all(0<=a.l.indexed_depth(i)<=100 for i in range(a.l.HOLE_COUNT))
