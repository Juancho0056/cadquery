"""Geometric section screening, not FEA or a material/load certificate.

Coordinates in mm, area in mm², inertia in mm⁴, section modulus in mm³.
Integrate thin slices using their actual tessellated surfaces. The two bending
axes are global Y and Z; no torsional constant or stress concentration claimed.
"""
from dataclasses import dataclass,asdict
import math
import cadquery as cq
import numpy as np
from modular_arm.v4.transition import finished_half
from modular_arm.v4.integrated_arm import axis_heights

@dataclass(frozen=True)
class Section:
    x_mm: float
    area_mm2: float
    centroid_y_mm: float
    centroid_z_mm: float
    iy_mm4: float
    iz_mm4: float
    iyz_mm4: float
    wy_mm3: float
    wz_mm3: float

def polygon_section(triangles):
    """Integrate oriented projected triangles on a planar section.
    Signed triangle integrals preserve holes. All triangles must have
    consistent orientation; a globally reversed normal is accepted.
    """
    t=np.asarray(triangles,dtype=float)
    if t.ndim!=3 or t.shape[1:]!=(3,2) or not np.isfinite(t).all():
        raise ValueError("invalid section triangles")
    y,z=t[:,:,0],t[:,:,1]
    a=((y[:,1]-y[:,0])*(z[:,2]-z[:,0])-(y[:,2]-y[:,0])*(z[:,1]-z[:,0]))/2
    if a.sum()<0:
        a=-a
    area=float(a.sum())
    if area<=1e-8:
        raise ValueError("empty section")
    cy=float(np.sum(a*y.sum(1)/3)/area)
    cz=float(np.sum(a*z.sum(1)/3)/area)
    y=y-cy
    z=z-cz
    iy=float(np.sum(a*(np.sum(z*z,axis=1)+z[:,0]*z[:,1]+z[:,1]*z[:,2]+z[:,2]*z[:,0])/6))
    iz=float(np.sum(a*(np.sum(y*y,axis=1)+y[:,0]*y[:,1]+y[:,1]*y[:,2]+y[:,2]*y[:,0])/6))
    iyz=float(np.sum(a*(y.sum(1)*z.sum(1)+np.sum(y*z,axis=1))/12))
    if min(iy,iz)<=0 or iy*iz-iyz*iyz<=0:
        raise ValueError("invalid section inertia")
    return area,cy,cz,iy,iz,iyz,iy/float(np.max(abs(z))),iz/float(np.max(abs(y)))

def measure(shape,x,*,thickness=.02):
    """Tessellate only planar cap at x-thickness/2 of a thin CAD slice."""
    if not math.isfinite(x) or not 0<thickness<=.1:
        raise ValueError("invalid section location or thickness")
    low=x-thickness/2
    slab=cq.Solid.makeBox(thickness,200,200,cq.Vector(low,-100,-100))
    cut=shape.intersect(slab)
    if not cut.Solids():
        raise ValueError(f"empty section at {x}")
    triangles=[]
    for f in cut.Faces():
        bb=f.BoundingBox()
        if bb.xlen>1e-6 or abs(bb.xmin-low)>1e-6:
            continue
        v,faces=f.tessellate(.025,.05)
        pts=np.array([[p.y,p.z] for p in v])
        triangles.extend(pts[np.asarray(faces)])
    if not triangles:
        raise ValueError(f"no planar cap at {x}")
    return Section(low,*polygon_section(triangles))

def screen(offset=15.4,stations=range(28,46),*,reinforcement=0.):
    """Compare actual machined neck cross sections before the STL join."""
    z=axis_heights(offset)
    result={}
    for index,sign in ((1,-1),(5,1)):
        shape=finished_half(index,axis_z=z[index],reinforcement=reinforcement if index==5 else 0.)
        rows=[measure(shape,sign*float(x)) for x in stations]
        if not rows:
            raise ValueError("no stations")
        result[str(index)]={"sections":[asdict(r) for r in rows],
             "minimum_area":asdict(min(rows,key=lambda r:r.area_mm2)),
             "minimum_wy":asdict(min(rows,key=lambda r:r.wy_mm3)),
             "minimum_wz":asdict(min(rows,key=lambda r:r.wz_mm3))}
    return {"offset_mm":offset,"b_reinforcement_mm":reinforcement,"scope":"CAD neck only; no load rating", "halves":result}
