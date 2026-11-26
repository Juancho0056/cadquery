import pytest
from modular_arm.assembly.common_selector import CommonSelectorConfig
from modular_arm.assembly.modular_segments import SegmentConfig,extension_parts

def test_distal_channel_is_single_solid():
    c=CommonSelectorConfig()
    m=SegmentConfig()
    p=extension_parts(c,m=m)["fixed_extension"]
    assert p.isValid() and len(p.Solids())==1
    assert p.BoundingBox().zlen==pytest.approx(m.distal_height_mm,abs=1e-6)
    assert p.Volume()<m.extension_length_mm*m.plate_width_mm*m.distal_height_mm

def test_mirrored_segment_volume():
    p=extension_parts(CommonSelectorConfig())
    assert p["fixed_extension"].Volume()==pytest.approx(p["moving_extension"].Volume(),abs=1e-6)

def test_invalid_wall():
    with pytest.raises(ValueError):
        SegmentConfig(distal_wall_mm=12.)
