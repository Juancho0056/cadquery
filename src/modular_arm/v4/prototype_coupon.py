"""Short, geometry-preserving joint coupons for unloaded fit tests."""
import cadquery as cq
from modular_arm.v4 import flush_arm

def parts():
    result={}
    for index,name in ((1,"arm_a_coupon"),(5,"arm_b_coupon")):
        shape=flush_arm.finished_flush(index)
        # Remove only the distal stub beyond X=38, leaving the joint and
        # its mating features unchanged. Cut face is the print bed.
        x=-38. if index==1 else -60.
        end=60. if index==1 else 38.
        shape=shape.intersect(cq.Solid.makeBox(end-x,120,120,cq.Vector(x,-60,-60))).clean()
        if not shape.isValid() or len(shape.Solids())!=1:
            raise ValueError("invalid coupon")
        result[name]=shape
    result["button"]=flush_arm.button()
    return result

def printable_parts():
    result={}
    for name,s in parts().items():
        if name!="button":
            s=s.rotate((0,0,0),(0,1,0),-90 if name=="arm_a_coupon" else 90)
        bb=s.BoundingBox()
        result[name]=s.translate((-bb.xmin,-bb.ymin,-bb.zmin))
    return result
