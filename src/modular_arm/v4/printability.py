"""Facet-based FDM overhang screening, not a slicer or support certificate."""
import math
import numpy as np

def overhang_report(triangles,build_axis,limit=45.,base_tolerance=.05):
    t=np.asarray(triangles,dtype=float)
    axis=np.asarray(build_axis,dtype=float)
    if t.ndim!=3 or t.shape[1:]!=(3,3) or not len(t) or not np.isfinite(t).all():
        raise ValueError("invalid triangles")
    if axis.shape!=(3,) or not np.isfinite(axis).all() or np.linalg.norm(axis)==0:
        raise ValueError("invalid build axis")
    if not 0<limit<90 or base_tolerance<0:
        raise ValueError("invalid limits")
    axis=axis/np.linalg.norm(axis)
    cross=np.cross(t[:,1]-t[:,0],t[:,2]-t[:,0])
    lengths=np.linalg.norm(cross,axis=1)
    good=lengths>1e-12
    heights=t@axis
    base=float(heights.min())
    on_bed=heights.max(1)<=base+base_tolerance
    # Vertical walls = 0 degrees; horizontal downward faces = 90.
    angles=np.degrees(np.arcsin(np.clip(-(cross[good]@axis)/lengths[good],0,1)))
    bad=(angles>limit)&(~on_bed[good])
    return {"limit_deg_from_vertical":limit,
            "unsupported_candidate_area_mm2":float(lengths[good][bad].sum()/2),
            "candidate_facets":int(bad.sum()),
            "max_downward_angle_deg":float(angles[~on_bed[good]].max()) if (~on_bed[good]).any() else 0.,
            "bed_face_area_mm2":float(lengths[on_bed].sum()/2)}
