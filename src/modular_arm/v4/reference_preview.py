"""Local-only original STL alignment and conservative section checks.
Preview is not a collision-certified assembly or a telescoping mechanism model.
"""
import math
import struct
from pathlib import Path
import numpy as np
from modular_arm.v4.reference_arm import read_part
from modular_arm.v4.reference_ends import AXIS_DISTANCE,A_CENTER_Z,B_CENTER_Z,parts

# Measured source coordinates of terminal bore axis, midpoint of mating gap.
CENTERS={1:(105.621437,153.206436,123.3),5:(146.49,151.486,10.)}

def aligned_triangles(archive,index,angle=0.):
    if index not in CENTERS:
        raise ValueError("only measured terminal parts supported")
    if not math.isfinite(angle):
        raise ValueError("invalid angle")
    p=read_part(archive,index)
    t=p.triangles.astype(float)-np.array(CENTERS[index])
    # Source +Z points toward the V4 fixed half for obj1, outward for obj5.
    q=t[..., [2,1,0]].copy()
    q[...,2]*=-1
    q+=np.array((-AXIS_DISTANCE,0,A_CENTER_Z) if index==1 else (AXIS_DISTANCE,0,B_CENTER_Z))
    if index==5:
        a=math.radians(angle)
        q=q@np.array([[math.cos(a),math.sin(a),0],[-math.sin(a),math.cos(a),0],[0,0,1]])
    return q

def slice_segments(triangles,x):
    pairs=[]
    for t in triangles[(triangles[:,:,0].min(1)<x)&(triangles[:,:,0].max(1)>x)]:
        points=[]
        for i in range(3):
            p,q=t[i],t[(i+1)%3]
            if (p[0]-x)*(q[0]-x)<0:
                points.append((p+(q-p)*(x-p[0])/(q[0]-p[0]))[1:])
        if len(points)==2:
            pairs.append(points)
    return np.asarray(pairs).reshape(-1,2,2)

def section_clearance(archive,index,x):
    """Distance from intended pin center to closest mesh-section boundary.
    Odd-even ray test checks if center lies in material, not merely proximity.
    """
    p=read_part(archive,index)
    seg=slice_segments(p.triangles,x)-np.array(CENTERS[index][1:])
    if not len(seg):
        raise ValueError("empty section")
    a,b=seg[:,0],seg[:,1]
    d=b-a
    f=np.clip(-np.sum(a*d,axis=1)/np.maximum(np.sum(d*d,axis=1),1e-20),0,1)
    distance=float(np.linalg.norm(a+f[:,None]*d,axis=1).min())
    crossing=(a[:,1]>0)!=(b[:,1]>0)
    aa,bb=a[crossing],b[crossing]
    inside=bool(np.count_nonzero(aa[:,0]-aa[:,1]*(bb[:,0]-aa[:,0])/(bb[:,1]-aa[:,1])>0)%2)
    return {"x":x,"center_in_material":inside,"centered_clear_diameter":0. if inside else 2*distance}

def terminal_report(archive):
    return {str(i):[section_clearance(archive,i,x) for x in xs]
            for i,xs in ((1,(100.5,103,105.6,108,110.5)),(5,(134,136,138,140,153,155,158,160)))}

def write_stl(path,triangles):
    t=np.asarray(triangles,dtype=np.float32)
    normal=np.cross(t[:,1]-t[:,0],t[:,2]-t[:,0])
    normal/=np.maximum(np.linalg.norm(normal,axis=1)[:,None],1e-20)
    records=np.zeros(len(t),dtype=[("n","<f4",3),("v","<f4",(3,3)),("a","<u2")])
    records["n"]=normal
    records["v"]=t
    with open(path,"wb") as f:
        f.write(b"V4 LOCAL PREVIEW - NOT FOR PRINTING".ljust(80,b" "))
        f.write(struct.pack("<I",len(t)))
        f.write(records.tobytes())

def export_preview(archive,destination,angle=0.,travel=0.):
    dest=Path(destination)
    dest.mkdir(parents=True,exist_ok=True)
    paths=[]
    for name,shape in parts(angle,travel).items():
        path=dest/(name+".stl")
        shape.exportStl(str(path),tolerance=.1,angularTolerance=.1)
        paths.append(path)
    for i in (1,5):
        path=dest/f"original_obj_{i}_aligned.stl"
        write_stl(path,aligned_triangles(archive,i,angle))
        paths.append(path)
    return paths
