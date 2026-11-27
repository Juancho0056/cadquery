"""Serial chain of reusable telescopic segments and canonical V4.1 joints.

Each segment uses one modified original inner and one modified original outer,
both with canonical D male ends. Free ends are male interfaces, not accessories.
Coordinates in mm. Initial same-handed chain; no load rating.
"""
import json,math
from pathlib import Path
import numpy as np
from modular_arm.v4 import extension_lock as l
e=l.e
m=e.m
ia=e.ia
LOCK_OFFSET=14.75
LOCK_PHASE=60.

def translation(x,y,z):
    t=np.eye(4)
    t[:3,3]=(x,y,z)
    return t

def rotation(angle):
    a=math.radians(angle)
    c,s=math.cos(a),math.sin(a)
    return np.array([[c,-s,0,0],[s,c,0,0],[0,0,1,0],[0,0,0,1]],float)

def segment_transform(index):
    if not isinstance(index,int) or not 0<=index<=12:
        raise ValueError("index must be 0..12")
    depth=l.indexed_depth(index)
    # Align the canonical A inner body to the original inner counterpart
    # of B. Both have the same source-to-local rotation.
    x=2*ia.JOIN_X+ia.CUTS[1]-ia.CUTS[5]+e.MOUTH_Z-depth
    z=m.flush_arm.AXES[5]-m.flush_arm.AXES[1]
    return translation(x,0.,z)

def frames(angles,indices):
    if not angles or len(indices)!=len(angles)+1:
        raise ValueError("N joints require N+1 segments")
    for idx in indices: segment_transform(idx)
    result=[np.eye(4)]
    for j,angle in enumerate(angles):
        if not math.isfinite(angle) or angle not in (0.,90.,180.,-90.,-180.):
            raise ValueError("unsupported sampled joint angle")
        if j<len(angles)-1:
            result.append(result[-1]@rotation(angle)@segment_transform(indices[j+1]))
    return result

def add(parts,name,mesh,transform):
    p=mesh.copy()
    p.apply_transform(transform)
    parts[name]=p

def assemble(archive,angles,indices,*,templates=None):
    f=frames(angles,indices)
    if templates is None:
        templates=(m.arm(archive,1),m.arm(archive,5),ia.original_mesh(archive,22))
    inner,outer,screw=templates
    parts={}
    # End segment: free B male end to first joint A.
    start=f[0]@np.linalg.inv(segment_transform(indices[0]))
    add(parts,"segment_0_outer",outer,start)
    add(parts,"segment_0_inner",inner,f[0])
    add(parts,"segment_0_lock",screw,start@l.screw_transform("b",l.indexed_depth(indices[0]),LOCK_OFFSET,LOCK_PHASE))
    for j,(angle,frame) in enumerate(zip(angles,f)):
        m.joint.parts(angle,0.)
        outgoing=frame@rotation(angle)
        for side,i,t in (("a",1,frame),("b",5,outgoing)):
            add(parts,f"joint_{j}_head_{side}",ia.cad_mesh(m.head(i)),t)
            for name,solid in m.connection_hardware(i).items():
                add(parts,f"joint_{j}_{side}_{name}",ia.cad_mesh(solid),t)
        add(parts,f"joint_{j}_button",ia.cad_mesh(m.flush_arm.button(0.)),frame)
        for name,solid in m.joint.screws(angle).items():
            add(parts,f"joint_{j}_{name}",ia.cad_mesh(solid),frame)
        idx=indices[j+1]
        next_frame=outgoing@segment_transform(idx)
        add(parts,f"segment_{j+1}_outer",outer,outgoing)
        add(parts,f"segment_{j+1}_inner",inner,next_frame)
        add(parts,f"segment_{j+1}_lock",screw,outgoing@l.screw_transform("b",l.indexed_depth(idx),LOCK_OFFSET,LOCK_PHASE))
    return parts,f

def scan(archive,destination):
    dest=Path(destination)
    dest.mkdir(parents=True,exist_ok=True)
    templates=(m.arm(archive,1),m.arm(archive,5),ia.original_mesh(archive,22))
    rows=[]
    cases=((0.,0.),(90.,0.),(0.,90.),(90.,-90.),(180.,0.),
           (0.,0.,0.),(90.,-90.,90.),(0.,180.,0.))
    for angles in cases:
        indices=(6,)*(len(angles)+1)
        parts,f=assemble(archive,angles,indices,templates=templates)
        defects=ia.audit(parts)
        row={"angles_deg":angles,"indices":indices,"segments":len(indices),
             "joint_origins_mm":[t[:3,3].tolist() for t in f],
             "collisions":defects,"clear":not defects}
        rows.append(row)
        print(json.dumps(row),flush=True)
        if all(a==0 for a in angles):
            preview=dest/f"preview_{len(indices)}_segments"
            preview.mkdir(exist_ok=True)
            for p in preview.glob("*.stl"): p.unlink()
            if not defects:
                for name,mesh in parts.items(): mesh.export(preview/(name+".stl"))
    result={"status":"serial_chain_candidate_only","rows":rows,
            "interface":{"male":"canonical D","engagement_mm":m.LENGTH,"hardware":"M4 x 35 nominal"},
            "limitations":"Same-handed joints accumulate 28.4 mm axis rise per joint in straight chain. No accessories, alternate handedness, swept motion, physical locking or structural/load qualification. Lock pose is discrete."}
    (dest/"report.json").write_text(json.dumps(result,indent=2),encoding="utf-8")
    return result
