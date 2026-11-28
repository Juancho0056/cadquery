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
    out={"elbow_"+n:cad_mesh(place(p)) for n,p in templates.items()}
    # Use canonical hardware envelopes, not substitute fastener dimensions.
    # Translate original B hardware to the compact B socket axis.
    dz=axis-j.old.s.f.AXES[5]
    for n,p in m.connection_hardware(5).items():
        out["hardware_"+n]=cad_mesh(place(p.translate(cq.Vector(0,0,dz))))
    return out

def write_scene(parts,path):
    scene=tm.Scene()
    for n,p in parts.items(): scene.add_geometry(p,geom_name=n,node_name=n)
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
    for angle in range(0,360,15):
        locked=angle%30==0
        base={"swivel_"+n:cad_mesh(p) for n,p in s.parts(angle,locked).items()}
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
                v=overlap(parts[x],parts[y])
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
