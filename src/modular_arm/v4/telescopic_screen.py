"""Original longitudinal interface screening; no assumed assembly certification."""
import numpy as np
from modular_arm.v4.reference_arm import read_part

CENTERS={1:(105.62144,153.20654),5:(146.39,151.505),3:(778.7141,151.75377)}
STATIONS={1:(.5,5,15,25,40,60,80,95,100,104,104.9),
          5:(40.1,41,45,46,50,55,60,70,80,100,120,140,145),
          3:(1,10,15,20,25,30,35,40,45,60,100,150,161)}

def section(triangles,center,z):
    """Parity and nearest boundary of an XY slice about a specified axis."""
    t=np.asarray(triangles,dtype=float)
    v=t[(t[:,:,2].min(1)<z)&(t[:,:,2].max(1)>z)]
    if not len(v):
        raise ValueError("empty section")
    if np.any(v[:,:,2]==z):
        raise ValueError("section through vertex; choose another station")
    b=np.roll(v,-1,axis=1)
    mask=(v[:,:,2]-z)*(b[:,:,2]-z)<0
    a,b=v[mask],b[mask]
    q=(a+(b-a)*((z-a[:,2])/(b[:,2]-a[:,2]))[:,None])[:,:2].reshape(-1,2,2)-center
    a,b=q[:,0],q[:,1]
    d=b-a
    den=np.einsum("ij,ij->i",d,d)
    k=np.divide(-np.einsum("ij,ij->i",a,d),den,out=np.zeros_like(den),where=den>0).clip(0,1)
    distance=float(np.linalg.norm(a+k[:,None]*d,axis=1).min())
    if distance<1e-6:
        raise ValueError("axis on boundary")
    mask=(a[:,1]>0)!=(b[:,1]>0)
    a,b=a[mask],b[mask]
    crossing=a[:,0]-a[:,1]*(b[:,0]-a[:,0])/(b[:,1]-a[:,1])
    occupied=bool(np.count_nonzero(crossing>0)%2)
    return {"z_mm":float(z),"axis_occupied":occupied,"nearest_boundary_mm":distance,
            "outer_radius_mm":float(np.linalg.norm(q,axis=2).max())}

def report(archive):
    result={}
    for i,stations in STATIONS.items():
        p=read_part(archive,i)
        result[str(i)]=[section(p.triangles,CENTERS[i],z+.00317) for z in stations]
    b=[r for r in result["5"] if 46<=r["z_mm"]<=140.01]
    shaft=[r for r in result["3"] if 20<=r["z_mm"]]
    if any(r["axis_occupied"] for r in b):
        clearance=None
    else:
        clearance=min(r["nearest_boundary_mm"] for r in b)-max(r["outer_radius_mm"] for r in shaft)
    return {"status":"sampled_reference_sections_only","parts":result,
            "candidate_b_radial_margin_mm":clearance,
            "limitations":"Original meshes only; discrete slices, not continuous travel, not verified mounting pose, not M4 assembly clearance."}
