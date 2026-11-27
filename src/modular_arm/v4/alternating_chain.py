"""Alternating joint frames, using canonical heads and a re-clocked inner tube.
Angles are local to each joint; inverted joints reverse world rotation sense.
No continuous-motion or load qualification.
"""
import json
from pathlib import Path
import numpy as np
from modular_arm.v4 import modular_chain as c, extension_clocking as clock
m,ia=c.m,c.ia

def frames(angles,indices):
    c.frames(angles,indices)  # canonical validation
    f=[np.eye(4)]
    for j in range(len(angles)-1):
        f.append(f[-1]@c.rotation(angles[j])@c.segment_transform(indices[j+1])@clock.module_transform(180.))
    return f

def templates(archive):
    return (m.arm(archive,1),m.arm(archive,5),ia.original_mesh(archive,22),
            clock.tube_a(archive,180.))

def assemble(archive,angles,indices,*,source=None):
    f=frames(angles,indices)
    inner,outer,screw,flipped_inner=templates(archive) if source is None else source
    parts={}
    start=np.linalg.inv(c.segment_transform(indices[0]))
    c.add(parts,"segment_0_outer",outer,start)
    c.add(parts,"segment_0_inner",inner,f[0])
    c.add(parts,"segment_0_lock",screw,start@c.l.screw_transform("b",c.l.indexed_depth(indices[0]),c.LOCK_OFFSET,c.LOCK_PHASE))
    for j,(angle,frame) in enumerate(zip(angles,f)):
        m.joint.parts(angle,0.)
        outgoing=frame@c.rotation(angle)
        for side,i,t in (("a",1,frame),("b",5,outgoing)):
            c.add(parts,f"joint_{j}_head_{side}",ia.cad_mesh(m.head(i)),t)
            for name,solid in m.connection_hardware(i).items():
                c.add(parts,f"joint_{j}_{side}_{name}",ia.cad_mesh(solid),t)
        c.add(parts,f"joint_{j}_button",ia.cad_mesh(m.flush_arm.button(0.)),frame)
        for name,solid in m.joint.screws(angle).items():
            c.add(parts,f"joint_{j}_{name}",ia.cad_mesh(solid),frame)
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
