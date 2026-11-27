"""Alternating joint frames, using canonical heads and a re-clocked inner tube.
Angles are local to each joint; inverted joints reverse world rotation sense.
No continuous-motion or load qualification.
"""
import json,math
from pathlib import Path
import numpy as np
from modular_arm.v4 import modular_chain as c, extension_clocking as clock
m,ia=c.m,c.ia

def frames(angles,indices):
    if not angles or len(indices)!=len(angles)+1:
        raise ValueError('N joints require N+1 segments')
    for idx in indices: c.segment_transform(idx)
    for angle in angles: validate_pose(angle,7.)
    f=[np.eye(4)]
    for j in range(len(angles)-1):
        f.append(f[-1]@c.rotation(angles[j])@c.segment_transform(indices[j+1])@clock.module_transform(180.))
    return f

def validate_pose(angle,travel):
    if not math.isfinite(angle) or abs(angle)>180:
        raise ValueError("invalid angle")
    if not math.isfinite(travel) or not 0<=travel<=m.joint.STROKE:
        raise ValueError("invalid travel")
    if travel<6.4 and abs(angle/45-round(angle/45))>1e-8:
        raise ValueError("nonindexed pose requires released clutch")

class JointMeshes:
    def __init__(self):
        self.heads={i:ia.cad_mesh(m.head(i)) for i in (1,5)}
        self.hardware={i:{n:ia.cad_mesh(s) for n,s in m.connection_hardware(i).items()} for i in (1,5)}
        self.retainers={n:ia.cad_mesh(s) for n,s in m.joint.screws(0.).items()}
        self.buttons={}
    def button(self,travel):
        if travel not in self.buttons:
            self.buttons[travel]=ia.cad_mesh(m.flush_arm.button(travel))
        return self.buttons[travel]

def templates(archive):
    return (m.arm(archive,1),m.arm(archive,5),ia.original_mesh(archive,22),
            clock.tube_a(archive,180.))

def assemble(archive,angles,indices,*,source=None,travels=None,joint_meshes=None):
    f=frames(angles,indices)
    travels=(0.,)*len(angles) if travels is None else travels
    if len(travels)!=len(angles): raise ValueError('one travel per joint')
    inner,outer,screw,flipped_inner=templates(archive) if source is None else source
    for angle,travel in zip(angles,travels): validate_pose(angle,travel)
    cache=JointMeshes() if joint_meshes is None else joint_meshes
    parts={}
    start=np.linalg.inv(c.segment_transform(indices[0]))
    c.add(parts,"segment_0_outer",outer,start)
    c.add(parts,"segment_0_inner",inner,f[0])
    c.add(parts,"segment_0_lock",screw,start@c.l.screw_transform("b",c.l.indexed_depth(indices[0]),c.LOCK_OFFSET,c.LOCK_PHASE))
    for j,(angle,frame) in enumerate(zip(angles,f)):
        outgoing=frame@c.rotation(angle)
        for side,i,t in (("a",1,frame),("b",5,outgoing)):
            c.add(parts,f"joint_{j}_head_{side}",cache.heads[i],t)
            for name,mesh in cache.hardware[i].items():
                c.add(parts,f"joint_{j}_{side}_{name}",mesh,t)
        c.add(parts,f"joint_{j}_button",cache.button(travels[j]),frame)
        for name,mesh in cache.retainers.items():
            c.add(parts,f"joint_{j}_{name}",mesh,outgoing)
        idx=indices[j+1]
        next_frame=outgoing@c.segment_transform(idx)
        internal=j<len(angles)-1
        c.add(parts,f"segment_{j+1}_outer",outer,outgoing)
        c.add(parts,f"segment_{j+1}_inner",flipped_inner if internal else inner,f[j+1] if internal else next_frame)
        c.add(parts,f"segment_{j+1}_lock",screw,outgoing@c.l.screw_transform("b",c.l.indexed_depth(idx),c.LOCK_OFFSET,c.LOCK_PHASE))
    return parts,f

def scan(archive,destination):
    dest=Path(destination); dest.mkdir(parents=True,exist_ok=True)
    source=templates(archive)
    rows=[]
    cases=((0.,0.),(90.,0.),(0.,90.),(90.,90.),(180.,0.),
           (0.,0.,0.),(90.,90.,90.),(0.,180.,0.))
    for angles in cases:
        indices=(6,)*(len(angles)+1)
        parts,f=assemble(archive,angles,indices,source=source)
        defects=ia.audit(parts)
        row={"local_angles_deg":angles,"indices":indices,"segments":len(indices),
             "joint_origins_mm":[t[:3,3].tolist() for t in f],
             "joint_axis_z":[float(t[2,2]) for t in f],
             "collisions":defects,"clear":not defects}
        rows.append(row); print(json.dumps(row),flush=True)
        if all(a==0 for a in angles):
            preview=dest/f"preview_{len(indices)}_segments"
            preview.mkdir(exist_ok=True)
            for p in preview.glob("*.stl"): p.unlink()
            if not defects:
                for name,mesh in parts.items(): mesh.export(preview/(name+".stl"))
    result={"status":"alternating_chain_candidate_only","rows":rows,
            "limitations":"Internal segments require alternate inner tube with canonical D key. End segments standard. Local joint angles reverse world sense on inverted joints. No accessories, swept motion or load qualification."}
    (dest/"report.json").write_text(json.dumps(result,indent=2),encoding="utf-8")
    return result
