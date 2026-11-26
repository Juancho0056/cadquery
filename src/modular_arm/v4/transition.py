"""Continuous outer silhouette and transverse envelope for integral V4 halves.

The circular plan arc is tangent to the cubic Bezier sides. The old neck is
not part of this construction. The axial envelope changes from a flat disc
to an almost circular 27.88 mm terminal. No claim of global G2 continuity.
"""
import math
import cadquery as cq

V=cq.Vector

def plan():
    # Tangent to circle radius 28 at x=16.8, y=22.4.
    p=V(16.8,22.4,-30)
    q=V(48,13.95,-30)
    pn=V(16.8,-22.4,-30)
    qn=V(48,-13.95,-30)
    edges=[
        cq.Edge.makeBezier([p,V(25.2,16.1,-30),V(36,13.95,-30),q]),
        cq.Edge.makeLine(q,qn),
        cq.Edge.makeBezier([qn,V(36,-13.95,-30),V(25.2,-16.1,-30),pn]),
        cq.Edge.makeThreePointArc(pn,V(-28,0,-30),p),
    ]
    return cq.Solid.extrudeLinear(cq.Wire.assembleEdges(edges),[],V(0,0,90))

def rounded_wire(x,w,h,z,r):
    """Rounded rectangular transverse profile; w/h are half extents."""
    def p(y,t):
        return V(x,y,z+t)
    a=[p(-w+r,-h),p(w-r,-h),p(w,-h+r),p(w,h-r),
       p(w-r,h),p(-w+r,h),p(-w,h-r),p(-w,-h+r)]
    d=r/math.sqrt(2)
    mid=[p(w-r+d,-h+r-d),p(w-r+d,h-r+d),
         p(-w+r-d,h-r+d),p(-w+r-d,-h+r-d)]
    es=[]
    for i in range(4):
        es.extend([cq.Edge.makeLine(a[2*i],a[2*i+1]),
                   cq.Edge.makeThreePointArc(a[2*i+1],mid[i],a[(2*i+2)%8])])
    return cq.Wire.assembleEdges(es)

def outer_blank(index, *, axis_z=None, reinforcement=0.):
    """Build the exterior before any journal, gear or retainer cuts."""
    if index not in (1,5):
        raise ValueError("unsupported reference body")
    if not math.isfinite(reinforcement) or not 0<=reinforcement<=6:
        raise ValueError("reinforcement must be 0..6 mm")
    if index==1 and reinforcement:
        raise ValueError("reinforcement is exclusive to B")
    bottom,top,z_end=(-10.,12.,4.) if index==1 else (12.4,26.4,19.4)
    if axis_z is not None:
        if not math.isfinite(axis_z) or not 4.<=axis_z<=19.4:
            raise ValueError('unsupported terminal axis height')
        z_end=axis_z
    z0=(bottom+top)/2
    h0=(top-bottom)/2
    def profile(x,t):
        return rounded_wire(x,22+(13.94-22)*t,h0+(13.94-h0)*t,
                            z0+(z_end-z0)*t,.5+(13.90-.5)*t)
    wires=[profile(27,0),profile(28,0),profile(32,.12),profile(38,.55),
           profile(44,.95),profile(46,1),profile(48,1)]
    start=28.
    if reinforcement:
        # Rebuild the transverse envelope, not an attachment to the old neck.
        # Additional height is biased upward; lower nominal profiles remain.
        def reinforced(x,t,lift):
            extra=reinforcement*lift
            return rounded_wire(x,22+(13.94-22)*t,h0+(13.94-h0)*t+extra/2,
                                z0+(z_end-z0)*t+extra/2,.5+(13.90-.5)*t)
        wires=[reinforced(x,t,lift) for x,t,lift in (
            (23,0,0),(24,0,0),(26,0,.45),(28,0,1),
            (32,.12,1),(38,.55,.5),(44,.95,0),(46,1,0),(48,1,0))]
        start=24.
    env=cq.Solid.makeLoft(wires,ruled=False)
    # Keep the exact disc height inside x=28. The envelope beyond it
    # is part of the original blank, not a reinforcement on an old neck.
    env=env.cut(cq.Solid.makeBox(100,200,200,V(start-100,-100,-100)))
    slab=cq.Solid.makeBox(58,100,top-bottom,V(-30,-50,bottom))
    s=plan().intersect(slab.fuse(env)).clean()
    if index==1:
        s=s.rotate((0,0,0),(0,0,1),180)
    if not s.isValid() or len(s.Solids())!=1:
        raise ValueError("invalid integral outer blank")
    return s

def finished_half(index, *, axis_z=None, reinforcement=0.):
    from modular_arm.v4 import joint
    b=outer_blank(index,axis_z=axis_z,reinforcement=reinforcement)
    return joint.fixed(b) if index==1 else joint.moving(b)
