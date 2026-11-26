import pytest
from modular_arm.config.ring import ExperimentalRingConfig


def test_default_pitch():
    c = ExperimentalRingConfig()
    assert c.tooth_count == 24
    assert c.pitch_degrees == 15.0
    assert c.tooth_radial_height_mm == 6.0


@pytest.mark.parametrize("overrides", [
    {"bore_diameter_mm": 60}, {"outer_diameter_mm": 50},
    {"thickness_mm": 0}, {"tooth_count": 2},
    {"tooth_fraction_of_pitch": 1},
])
def test_rejects_invalid_parameters(overrides):
    with pytest.raises(ValueError):
        ExperimentalRingConfig(**overrides)
