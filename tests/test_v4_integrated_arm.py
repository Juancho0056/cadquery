import os
import pytest

pytest.importorskip("trimesh")
pytest.importorskip("manifold3d")
from modular_arm.v4.integrated_arm import integrated_half,parts,audit

@pytest.fixture(scope="module")
def archive():
    a=os.environ.get("ARM_REFERENCE_ZIP")
    if not a:
        pytest.skip("set ARM_REFERENCE_ZIP")
    return a

@pytest.mark.parametrize("index",[1,5])
def test_integral_body(archive,index):
    m=integrated_half(archive,index)
    assert m.is_volume
    assert len(m.split(only_watertight=False))==1

@pytest.mark.parametrize("angle,travel",[(0,0),(45,0),(90,0),(30,7)])
def test_integral_pose(archive,angle,travel):
    p=parts(archive,angle,travel)
    assert audit(p)==[]

@pytest.mark.parametrize("index",[1,5])
def test_original_longitudinal_preservation(archive,index):
    from modular_arm.v4.integrated_arm import original_mesh,crop_terminal,align_body,preservation_report
    original=align_body(crop_terminal(original_mesh(archive,index),index),index)
    report=preservation_report(original,integrated_half(archive,index),index)
    assert report["added_mm3"]<=.05
    assert report["removed_mm3"]<=.05

def test_export_separates_hardware(archive,tmp_path):
    from modular_arm.v4.integrated_arm import export,PRINTABLE
    import json
    export(archive,tmp_path)
    assert {p.stem for p in (tmp_path/"prototype_printable").glob("*.stl")}==set(PRINTABLE)
    assert len(list((tmp_path/"assembly_preview").glob("retainer_*.stl")))==3
    manifest=json.loads((tmp_path/"manifest.json").read_text())
    assert manifest["status"]=="experimental_not_load_rated"

@pytest.mark.parametrize("index",[1,5])
def test_reinforced_core_valid(index):
    from modular_arm.v4.integrated_arm import joint_half
    from modular_arm.v4 import joint
    import cadquery as cq
    s=joint_half(index)
    original=joint.fixed() if index==1 else joint.moving()
    core=cq.Solid.makeCylinder(28,60,cq.Vector(0,0,-15))
    assert s.isValid() and len(s.Solids())==1
    assert original.intersect(core).cut(s).Volume()<1e-6
    assert s.intersect(core).cut(original).Volume()<1e-6
