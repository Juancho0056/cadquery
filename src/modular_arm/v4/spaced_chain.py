"""Compose locally screened 29.4 mm joints into an alternating chain."""
import json
from pathlib import Path
import numpy as np
from modular_arm.v4 import separation_variant as s
from modular_arm.v4.chain_motion_screen import save
a,c,ia=s.a,s.c,s.ia
DELTA=1.

def offset(index):
    return c.translation(0.,0.,s.shift(index,DELTA))

def frames(angles,indices):
    if not angles or len(indices)!=len(angles)+1:
        raise ValueError("N joints require N+1 segments")
    for i in indices: c.segment_transform(i)
    result=[np.eye(4)]
    for j,angle in enumerate(angles):
        a.validate_pose(angle,7.)
        if j<len(angles)-1:
            result.append(result[-1]@c.rotation(angle)@offset(5)
                          @c.segment_transform(indices[j+1])
                          @s.clock.module_transform(180.)@np.linalg.inv(offset(1)))
    return result

class Builder:
    def __init__(self,archive):
        self.archive=archive
        self.source=a.templates(archive)
        self.cache=a.JointMeshes()
        self.heads={i:ia.cad_mesh(s.head(i,DELTA)) for i in (1,5)}
        self.buttons={t:ia.cad_mesh(s.button(DELTA,t)) for t in (0.,7.)}
    def parts(self,angles,indices):
        f=frames(angles,indices)
        parts={}
        for j,(angle,frame) in enumerate(zip(angles,f)):
            travel=0. if abs(angle/45-round(angle/45))<1e-8 else 7.
            local=s.build(self.archive,DELTA,indices[j:j+2],angle,travel,
                          source=self.source,cache=self.cache,heads=self.heads,buttons=self.buttons)
            for name,mesh in local.items():
                if name.startswith("joint_0_"):
                    key=f"joint_{j}_"+name[len("joint_0_"):]
                elif name.startswith("segment_0_"):
                    suffix=name[len("segment_0_"):]
                    if j and suffix!="inner": continue
                    key=f"segment_{j}_"+suffix
                elif name.startswith("segment_1_"):
                    suffix=name[len("segment_1_"):]
                    if j<len(angles)-1 and suffix=="inner": continue
                    key=f"segment_{j+1}_"+suffix
                else: raise ValueError(name)
                if key in parts: raise ValueError("duplicate "+key)
                c.add(parts,key,mesh,frame)
        return parts,f

def scan(archive,destination):
    dest=Path(destination); dest.mkdir(parents=True,exist_ok=True)
    result={"status":"spaced_alternating_chain_samples","complete":False,"axis_spacing_mm":29.4,"rows":[],
            "limitations":"Discrete whole-chain intersections only; no positive clearance, continuous folding, physical locking or load rating."}
    path=dest/"report.json"
    save(path,result)
    builder=Builder(archive)
    cases=[((0.,0.),(6,6,6)),((180.,0.),(6,6,6)),((0.,180.),(6,6,6)),
           ((0.,0.,0.),(6,6,6,6)),((90.,90.,90.),(6,6,6,6)),
           ((0.,180.,0.),(6,6,6,6)),((0.,180.,0.),(6,0,12,6)),
           ((0.,180.,0.),(6,12,0,6))]
    for n,(angles,indices) in enumerate(cases):
        parts,f=builder.parts(angles,indices)
        defects=ia.audit(parts)
        row={"angles_deg":angles,"indices":indices,"collisions":defects,"clear":not defects,
             "joint_origins_mm":[t[:3,3].tolist() for t in f]}
        result["rows"].append(row); save(path,result)
        print(f"[{n+1}/{len(cases)}] "+json.dumps(row),flush=True)
        preview=dest/f"preview_case_{n+1}"
        if preview.exists():
            for old in preview.glob("*.stl"): old.unlink()
        if not defects:
            preview.mkdir(exist_ok=True)
            for name,mesh in parts.items(): mesh.export(preview/(name+".stl"))
    result["complete"]=True; save(path,result)
    return result
