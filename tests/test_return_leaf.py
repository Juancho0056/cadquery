import pytest
from modular_arm.mechanisms.installed_return import ReturnConfig,return_leaf,return_deflection,beam_strain_estimate

@pytest.mark.parametrize("angle",[0,5,10,15,20])
def test_prescribed_leaf_geometry(angle):
    c=ReturnConfig()
    d=return_deflection(c,16.,angle)
    leaf=return_leaf(c,deflection=d)
    assert leaf.isValid() and len(leaf.Solids())==1
    assert leaf.Volume()>0

def test_preload_and_screening():
    c=ReturnConfig()
    assert return_deflection(c,16.,0)==c.preload_mm
    assert beam_strain_estimate(c,return_deflection(c,16.,20))>0
    assert return_deflection(c,16.,20)>return_deflection(c,16.,0)
