from itertools import combinations
import pytest
from modular_arm.mechanisms.indexing import IndexingConfig, indexing_parts, posed_pawl, make_indexed_rotor

@pytest.fixture(scope="module")
def cfg():
    return IndexingConfig()

@pytest.fixture(scope="module")
def rotor(cfg):
    return make_indexed_rotor(cfg)

def test_geometry_and_static_pairs(cfg):
    parts = indexing_parts(cfg)
    for name,p in parts.items():
        assert p.isValid() and len(p.Solids()) == 1,name
    for (a,p),(b,q) in combinations(parts.items(),2):
        assert p.intersect(q).Volume()<1e-6,(a,b)

@pytest.mark.parametrize("lower",[False,True])
def test_pawl_retraction(cfg,rotor,lower):
    for i in range(6):
        assert posed_pawl(cfg,cfg.release_degrees*i/5,lower).intersect(rotor).Volume()<1e-6

@pytest.mark.parametrize("lower",[False,True])
def test_released_pitch_sweep(cfg,rotor,lower):
    p=posed_pawl(cfg,cfg.release_degrees,lower)
    for i in range(6):
        angle=cfg.ring.pitch_degrees*i/5
        assert p.intersect(rotor.rotate((0,0,0),(0,0,1),angle)).Volume()<1e-6

@pytest.mark.parametrize("lower",[False,True])
def test_fixed_pawl_blocks_both_directions(cfg,rotor,lower):
    p=posed_pawl(cfg,0,lower)
    for sign in (-1,1):
        assert p.intersect(rotor.rotate((0,0,0),(0,0,1),sign*cfg.ring.pitch_degrees/5)).Volume()>1e-3
