"""Integrate unchanged compact swivel with original local clamp and V4.5 elbow.
Static sampled review only; no load approval or third-party redistribution.
"""
import argparse, hashlib, json, subprocess
from pathlib import Path
from itertools import combinations
import numpy as np
import trimesh as tm
import cadquery as cq
from build_v45_clamp_assembly import original_mesh, cad_mesh, overlap
from modular_arm.v45 import clamp_swivel_compact as s, joint as j
from modular_arm.v4 import modular_connection as m

def provenance(archive):
    def git(*args):
        return subprocess.check_output(["git",*args],text=True).strip()
    return {"commit":git("rev-parse","HEAD"),"dirty":bool(git("status","--porcelain")),
            "archive_sha256":hashlib.sha256(Path(archive).read_bytes()).hexdigest()}

def clamp_parts(archive):
    body=original_mesh(archive,2,"main clamp body")
    section=body.section(plane_normal=[0,1,0],plane_origin=[0,53.9,0])
    if section is None: raise ValueError("socket section missing")
    loops=[p for p in section.discrete if 27.9<np.ptp(p[:,0])<28.1 and 27.9<np.ptp(p[:,2])<28.1]
    if len(loops)!=1: raise ValueError("ambiguous socket")
    loop=loops[0]
    center=(loop.min(axis=0)+loop.max(axis=0))/2
    body.apply_translation([-center[0],-54,-center[2]])
    body.apply_transform(tm.transformations.rotation_matrix(np.pi/2,[1,0,0]))
    screw=original_mesh(archive,19,"attachment locking screw")
    screw.apply_transform(tm.transformations.rotation_matrix(-np.pi/2,[1,0,0]))
    screw.apply_translation([0,-(s.a.STEM_D/2-3.2)-screw.bounds[1,1],-s.a.LOCK_DEPTH])
    return {"clamp_body":body,"clamp_screw_trial":screw},center

def elbow(angle,templates):
    axis=j.old.s.f.AXES[5]+j.old.s.shift(5,1.)
    # B socket mouth x=JOIN maps to z=D_Z; inward -x maps upward.
    # Flat +z of socket maps to +x of canonical D tenon.
    def place(shape):
        shape=shape.rotate((0,0,0),(0,1,0),90)
        shape=shape.translate(cq.Vector(-axis,0,s.D_Z+m.JOIN))
        return s.turn(shape,angle)
    out={"elbow_"+n:place(p) for n,p in templates.items() if n!="key"}
    # Use canonical hardware envelopes, not substitute fastener dimensions.
    # Translate original B hardware to the compact B socket axis.
    dz=axis-j.old.s.f.AXES[5]
    for n,p in m.connection_hardware(5).items():
        out["hardware_"+n]=place(p.translate(cq.Vector(0,0,dz)))
    return out

def mesh_for_boolean(shape):
    attempts=[]
    for linear,angular in ((.025,.08),(.05,.12),(.0125,.06),(.1,.15)):
        v,f=shape.copy(mesh=False).tessellate(linear,angular)
        mesh=tm.Trimesh(vertices=[p.toTuple() for p in v],faces=f,process=True)
        tm.repair.fix_normals(mesh,multibody=True)
        error=abs(mesh.volume-shape.Volume())/max(shape.Volume(),1e-9)
        attempts.append({"linear":linear,"angular":angular,"watertight":bool(mesh.is_watertight),
                         "winding":bool(mesh.is_winding_consistent),"relative_volume_error":float(error)})
        if mesh.is_volume and error<=.002:
            return mesh,attempts
    return None,attempts

def write_scene(parts,path):
    scene=tm.Scene()
    for n,p in parts.items(): scene.add_geometry(p if isinstance(p,tm.Trimesh) else cad_mesh(p),geom_name=n,node_name=n)
    path.write_bytes(scene.export(file_type="3mf"))

def build(archive,folder):
    folder=Path(folder);folder.mkdir(parents=True,exist_ok=True)
    report={"status":"REVIEW_ONLY","passed":False,"checks":[],
            "provenance":provenance(archive),"load":"NOT_RUN",
            "cap_reverse_retention":"NOT_RUN_NO_POSITIVE_STOP",
            "thread_fit":"NOT_RUN","clamp_jaw_assembly":"NOT_RUN",
            "continuous_motion":"NOT_RUN"}
    path=folder/"report.json"
    def save(): path.write_text(json.dumps(report,indent=2),encoding="utf-8")
    save()
    clamp,center=clamp_parts(archive)
    report["socket_center_source_xz_mm"]=[float(center[0]),float(center[2])]
    templates=j.templates("magnet")
    report["mesh_diagnostics"]={}
    mesh_cache={}
    def checked_mesh(name,shape):
        if name not in mesh_cache:
            mesh,attempts=mesh_for_boolean(shape)
            mesh_cache[name]=mesh
            report["mesh_diagnostics"][name]=attempts
        mesh=mesh_cache[name]
        if mesh is None: raise ValueError(f"No closed tessellation: {name}")
        return mesh
    for angle in range(0,360,15):
        locked=angle%30==0
        base={"swivel_"+n:p for n,p in s.parts(angle,locked).items()}
        head=elbow(angle,templates)
        parts={**clamp,**base,**head}
        if angle in (0,15,90,180,270):
            write_scene(parts,folder/f"REVIEW_{angle:03d}_NO_IMPRIMIR.3mf")
        pairs=[(x,y) for x in base for y in (*clamp,*head)]
        pairs.extend((x,y) for x in clamp for y in head)
        # Trial threaded screw/body contact deliberately excluded; no thread-fit claim.
        for x,y in pairs:
            row={"angle":angle,"locked":locked,"pair":[x,y]}
            try:
                a,b=parts[x],parts[y]
                if not isinstance(a,tm.Trimesh) and not isinstance(b,tm.Trimesh):
                    v=a.intersect(b).Volume()
                    row["engine"]="CAD_BREP"
                else:
                    def mesh(n,p):
                        if isinstance(p,tm.Trimesh): return p
                        key=n if n in ("swivel_fixed","swivel_cap","swivel_pin_key") else f"{n}@{angle}"
                        return checked_mesh(key,p)
                    v=overlap(mesh(x,a),mesh(y,b))
                    row["engine"]="MESH_MANIFOLD"
                row.update(intersection_mm3=v,status="PASS" if v<=.05 else "FAIL")
            except Exception as e:
                row.update(status="NOT_RUN",reason=str(e))
            report["checks"].append(row)
        save()
    report["passed"]=bool(report["checks"]) and all(c["status"]=="PASS" for c in report["checks"])
    save()
    return report

if __name__=="__main__":
    p=argparse.ArgumentParser()
    p.add_argument("archive")
    p.add_argument("--output",default="out/v45_clamp_swivel_integration")
    args=p.parse_args()
    r=build(args.archive,args.output)
    failures=[c for c in r["checks"] if c["status"]!="PASS"]
    print(json.dumps({"passed":r["passed"],"checks":len(r["checks"]),"failures":failures},indent=2))
    if not r["passed"]: raise SystemExit(1)
