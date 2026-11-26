from zipfile import ZipFile
import pytest
from modular_arm.validation.three_mf import inventory

MODEL = """<model xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02" unit="inch">
<resources><object id="1"><mesh><vertices><vertex x="0" y="0" z="0"/>
<vertex x="1" y="0" z="0"/><vertex x="0" y="1" z="0"/></vertices>
<triangles><triangle v1="0" v2="1" v3="2"/></triangles></mesh></object></resources>
<build><item objectid="1" transform="1 0 0 0 1 0 0 0 1 10 0 0"/></build></model>"""


def make_file(tmp_path, xml=MODEL):
    path = tmp_path / "fixture.3mf"
    with ZipFile(path, "w") as archive:
        archive.writestr("3D/3dmodel.model", xml)
    return path


def test_inventory_preserves_local_dimensions_and_build_transform(tmp_path):
    result = inventory(make_file(tmp_path))
    model = result["models"][0]
    assert model["objects"][0]["local_bounds_mm"]["x"] == [0, 25.4]
    assert model["objects"][0]["triangles"] == 1
    assert model["build_items"][0]["transform"].endswith("10 0 0")
    assert len(result["sha256"]) == 64
    assert result["mechanical_validation"] == "NOT_RUN"


def test_rejects_missing_vertex(tmp_path):
    with pytest.raises(ValueError, match="missing vertex"):
        inventory(make_file(tmp_path, MODEL.replace('v3="2"', 'v3="9"')))


def test_rejects_unknown_unit(tmp_path):
    with pytest.raises(ValueError, match="unsupported unit"):
        inventory(make_file(tmp_path, MODEL.replace('unit="inch"', 'unit="unknown"')))


def test_rejects_nonfinite_vertex(tmp_path):
    with pytest.raises(ValueError, match="non-finite"):
        inventory(make_file(tmp_path, MODEL.replace('x="1"', 'x="nan"')))


def test_rejects_empty_archive(tmp_path):
    path = tmp_path / "empty.3mf"
    with ZipFile(path, "w"):
        pass
    with pytest.raises(ValueError, match="no model"):
        inventory(path)
