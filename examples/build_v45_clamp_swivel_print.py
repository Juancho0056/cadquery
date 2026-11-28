"""Fail-closed export of five independent unloaded fit-prototype parts."""
import argparse, json, math, tempfile, shutil, hashlib, subprocess
from pathlib import Path
import cadquery as cq
import trimesh as tm
from modular_arm.v45 import clamp_swivel_compact as s
from build_v45_clamp_swivel_integration import mesh_for_boolean

NAMES=("fixed","rotor","cap","pin","pin_key")
def orient(name,shape):
    # Cap cosmetic face on bed; rotor D upward; pin longitudinal axis in XY.
    if name=="cap": shape=shape.rotate((0,0,0),(1,0,0),180)
    if name=="pin_key": shape=shape.rotate((0,0,0),(0,1,0),90)
    b=shape.BoundingBox()
    return shape.translate(cq.Vector(-b.xmin,-b.ymin,-b.zmin))

def validate_mesh(mesh,volume):
    if not isinstance(mesh,tm.Trimesh) or not mesh.is_volume:
        raise ValueError("mesh is not a closed oriented volume")
    if len(mesh.split(only_watertight=False))!=1:
        raise ValueError("mesh has multiple components")
    if abs(mesh.volume-volume)/volume>.002:
        raise ValueError("mesh volume differs from CAD by more than 0.2%")

def export(folder):
    folder=Path(folder)
    folder.mkdir(parents=True,exist_ok=True)
    dest=folder/"prototype_printable"
    if dest.exists(): shutil.rmtree(dest)
    report={"status":"BLOCKED","passed":False,"parts":{},
            "commit":subprocess.check_output(["git","rev-parse","HEAD"],text=True).strip(),
            "scope":"unloaded fit only; no load, vibration or reverse-cap retention approval"}
    def save(): (folder/"print_report.json").write_text(json.dumps(report,indent=2),encoding="utf-8")
    save()
    audit=s.audit()
    report["geometry_checks"]=len(audit["checks"])
    report["geometry_passed"]=audit["passed"]
    if not audit["passed"]:
        report["geometry_failures"]=[r for r in audit["checks"] if r["status"]!="PASS"]
        save()
        return report
    with tempfile.TemporaryDirectory(dir=folder) as tmp:
        tmp=Path(tmp)
        for name,original in s.parts().items():
            row=report["parts"][name]={}
            try:
                shape=orient(name,original)
                volume=shape.Volume()
                mesh,attempts=mesh_for_boolean(shape)
                row["attempts"]=attempts
                if mesh is None:
                    # Alternate native STL triangulation; re-read and certify.
                    # No arbitrary hole filling or voxel remeshing.
                    for tol,ang in ((.02,.08),(.04,.12),(.08,.15)):
                        test=tmp/"candidate.stl"
                        cq.exporters.export(shape,str(test),tolerance=tol,angularTolerance=ang)
                        candidate=tm.load_mesh(str(test),process=True)
                        tm.repair.fix_normals(candidate,multibody=True)
                        try: validate_mesh(candidate,volume)
                        except ValueError as e:
                            attempts.append({"native_stl":tol,"angular":ang,"reason":str(e)})
                            continue
                        mesh=candidate
                        break
                validate_mesh(mesh,volume)
                mesh.apply_translation(-mesh.bounds[0])
                mesh.export(tmp/(name+".stl"))
                scene=tm.Scene(mesh)
                (tmp/(name+".3mf")).write_bytes(scene.export(file_type="3mf"))
                cq.exporters.export(shape,str(tmp/(name+".step")))
                # Verify actual serialized meshes, not merely in-memory operands.
                for ext in ("stl","3mf"):
                    loaded=tm.load(str(tmp/(name+"."+ext)),force="mesh",process=True)
                    validate_mesh(loaded,volume)
                row.update(status="PASS",cad_volume_mm3=volume,mesh_volume_mm3=float(mesh.volume),
                           size_mm=mesh.extents.tolist(),
                           sha256={e:hashlib.sha256((tmp/(name+"."+e)).read_bytes()).hexdigest() for e in ("stl","3mf","step")})
            except Exception as exc:
                row.update(status="FAIL",reason=str(exc))
            save()
        if set(report["parts"])==set(NAMES) and all(r["status"]=="PASS" for r in report["parts"].values()):
            dest.mkdir()
            for n in NAMES:
                for ext in ("stl","3mf","step"):
                    shutil.copy2(tmp/(n+"."+ext),dest)
            report.update(status="UNLOADED_FIT_PROTOTYPE_ONLY",passed=True)
    save()
    return report

if __name__=="__main__":
    p=argparse.ArgumentParser()
    p.add_argument("--output",default="out/v45_clamp_swivel_print")
    args=p.parse_args()
    r=export(args.output)
    print(json.dumps(r,indent=2))
    if not r["passed"]: raise SystemExit(1)
