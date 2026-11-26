"""Compare print orientations without changing any functional geometry."""
import math
import numpy as np
from modular_arm.v4.printability import overhang_report,regional_overhang

def compare(triangles,direction,limit=45.):
    if direction not in (-1,1):
        raise ValueError("direction must be +/-1")
    axes=[("vertical",(-direction,0,0)),("flat_positive_z",(0,0,1)),
          ("flat_negative_z",(0,0,-1)),("side_positive_y",(0,1,0)),
          ("side_negative_y",(0,-1,0))]
    for degrees in (15,30,45):
        a=math.radians(degrees)
        for name,y,z in (("up",0,1),("down",0,-1),("left",1,0),("right",-1,0)):
            axes.append((f"tilt_{degrees}_{name}",(-direction*math.cos(a),y*math.sin(a),z*math.sin(a))))
    t=np.asarray(triangles,dtype=float)
    result=[]
    for name,axis in axes:
        r=overhang_report(t,axis,limit)
        ids=r.pop("candidate_indices")
        r["regions"]=regional_overhang(t,ids,direction=direction)
        heights=t@np.asarray(axis)
        r.update(name=name,build_axis=list(axis),height_mm=float(heights.max()-heights.min()))
        result.append(r)
    return result
