"""Review-only assembly from local original clamp archive and canonical V4.5 CAD.
Never publishes third-party meshes. No load qualification.
"""
import argparse, json, zipfile
from pathlib import Path
from xml.etree import ElementTree as ET
import numpy as np
import trimesh as tm
import cadquery as cq
from modular_arm.v45 import clamp_adapter as a, joint as j

def original_mesh(archive, object_id, expected_name):
    with zipfile.ZipFile(archive) as z:
        cfg=ET.fromstring(z.read("Metadata/model_settings.config"))
        meta=cfg.find(f"./object[@id='{object_id}']/metadata[@key='name']")
        if meta is None or meta.get("value")!=expected_name:
            raise ValueError("Unexpected source object; use original clamp archive")
        root=ET.fromstring(z.read("3D/3dmodel.model"))
        obj=root.find(f".//{{*}}object[@id='{object_id}']")
        component=obj.find(".//{*}component")
        if component.get("transform")!="1 0 0 0 1 0 0 0 1 0 0 0":
            raise ValueError("Unexpected source transform")
        path=next(v for k,v in component.attrib.items() if k.endswith("}path"))
        mesh=ET.fromstring(z.read(path.lstrip("/")))
        v=[[float(p.get(k)) for k in "xyz"] for p in mesh.findall(".//{*}vertex")]
        f=[[int(p.get(k)) for k in ("v1","v2","v3")] for p in mesh.findall(".//{*}triangle")]
    return tm.Trimesh(vertices=v,faces=f,process=True)

def cad_mesh(shape):
    v,f=shape.tessellate(.025,.08)
    return tm.Trimesh(vertices=[p.toTuple() for p in v],faces=f,process=True)

def overlap(a,b):
    if not a.is_volume or not b.is_volume:
        raise ValueError("Boolean requires oriented closed solids")
    return abs(float(tm.boolean.intersection([a,b],engine="manifold").volume))

def build(archive, folder):
    folder=Path(folder); folder.mkdir(parents=True,exist_ok=True)
    clamp=original_mesh(archive,2,"main clamp body")
    section=clamp.section(plane_normal=[0,1,0],plane_origin=[0,53.9,0])
    loops=[p for p in section.discrete if 27.9<np.ptp(p[:,0])<28.1 and 27.9<np.ptp(p[:,2])<28.1]
    if len(loops)!=1: raise ValueError("Socket section not identified")
    loop=loops[0]; center=(loop.min(axis=0)+loop.max(axis=0))/2
    clamp.apply_translation([-center[0],-54,-center[2]])
    clamp.apply_transform(tm.transformations.rotation_matrix(np.pi/2,[1,0,0]))
    screw=original_mesh(archive,19,"attachment locking screw")
    # Trial tip position: 3.2 mm inside stem surface. Not a thread fit claim.
    screw.apply_transform(tm.transformations.rotation_matrix(-np.pi/2,[1,0,0]))
    screw.apply_translation([0,-(a.STEM_D/2-3.2)-screw.bounds[1,1],-a.LOCK_DEPTH])
    axis=j.old.s.f.AXES[5]+j.old.s.shift(5,1.)
    def place(s):
        return s.rotate((0,0,0),(0,1,0),90).translate((-axis,0,a.FLANGE_H+a.NECK_H+ a.m.JOIN))
    parts={"clamp_original":clamp,"adapter":cad_mesh(a.build()),"locking_screw_trial":screw}
    for name,shape in j.parts(0.,0.,j.templates("magnet")).items():
        parts["joint_"+name]=cad_mesh(place(shape))
    scene=tm.Scene()
    for name,mesh in parts.items(): scene.add_geometry(mesh,geom_name=name,node_name=name)
    (folder/"REVIEW_NO_IMPRIMIR.3mf").write_bytes(scene.export(file_type="3mf"))
    report={"status":"REVIEW_ONLY","socket_center_source_xz_mm":[float(center[0]),float(center[2])],
            "socket_diameter_mm":float(np.ptp(loop[:,0])),
            "checks":{},"physical_load":"NOT_RUN","thread_fit":"NOT_RUN","retention":"NOT_RUN"}
    pairs=[("clamp_original","adapter"),("adapter","joint_head_b"),("clamp_original","joint_head_b"),
           ("adapter","locking_screw_trial")]
    for x,y in pairs:
        try:
            vol=overlap(parts[x],parts[y])
            report["checks"][x+"/"+y]={"intersection_mm3":vol,"status":"PASS" if vol<=.05 else "FAIL"}
        except Exception as e:
            report["checks"][x+"/"+y]={"status":"NOT_RUN","reason":str(e)}
    (folder/"report.json").write_text(json.dumps(report,indent=2),encoding="utf-8")
    return report

if __name__=="__main__":
    p=argparse.ArgumentParser()
    p.add_argument("archive"); p.add_argument("--output",default="out/v45_clamp_assembly")
    args=p.parse_args()
    print(json.dumps(build(args.archive,args.output),indent=2))
