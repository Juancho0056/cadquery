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

def roof_height(x,axis_z,reinforcement):
    """Bounded smoothstep roof, flat at both ends of each interval."""
    knots=((24.,26.4),(32.,26.4+reinforcement),
           (46.,axis_z+13.94),(48.,axis_z+13.94))
    if x<=knots[0][0]:
        return knots[0][1]
    for (a,u),(b,v) in zip(knots,knots[1:]):
        if x<=b:
            q=(x-a)/(b-a)
            return u+(v-u)*q*q*(3-2*q)
    return knots[-1][1]

def roof_envelope(axis_z,reinforcement):
    """Extruded XZ silhouette, exact cubic segments without spline overshoot."""
    knots=((-30.,26.4),(24.,26.4),(32.,26.4+reinforcement),
           (46.,axis_z+13.94),(49.,axis_z+13.94))
    def p(x,z):
        return V(x,-50,z)
    es=[]
    for (a,u),(b,v) in zip(knots,knots[1:]):
        d=(b-a)/3
        es.append(cq.Edge.makeBezier([p(a,u),p(a+d,u),p(b-d,v),p(b,v)]))
    es.extend([cq.Edge.makeLine(p(49,knots[-1][1]),p(49,-30)),
               cq.Edge.makeLine(p(49,-30),p(-30,-30)),
               cq.Edge.makeLine(p(-30,-30),p(-30,26.4))])
    return cq.Solid.extrudeLinear(cq.Wire.assembleEdges(es),[],V(0,100,0))

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
        # Distribute height over a longer span. The exact roof envelope
        # below clips interpolation overshoot rather than trusting loft knots.
        def reinforced(x,t):
            bottom_z=z0+(z_end-z0)*t-(h0+(13.94-h0)*t)
            upper_z=roof_height(x,z_end,reinforcement)
            h=(upper_z-bottom_z)/2
            r=min(.5+(13.90-.5)*t,h-.04)
            return rounded_wire(x,22+(13.94-22)*t,h,
                                (upper_z+bottom_z)/2,r)
        wires=[reinforced(x,t) for x,t in (
            (23,0),(24,0),(26,0),(28,0),(30,.05),(32,.12),
            (35,.30),(38,.55),(41,.78),(44,.95),(46,1),(48,1))]
        start=24.
    env=cq.Solid.makeLoft(wires,ruled=False)
    # Keep the exact disc height inside x=28. The envelope beyond it
    # is part of the original blank, not a reinforcement on an old neck.
    env=env.cut(cq.Solid.makeBox(100,200,200,V(start-100,-100,-100)))
    slab=cq.Solid.makeBox(58,100,top-bottom,V(-30,-50,bottom))
    s=plan().intersect(slab.fuse(env)).clean()
    if reinforcement:
        s=s.intersect(roof_envelope(z_end,reinforcement)).clean()
    if index==1:
        s=s.rotate((0,0,0),(0,0,1),180)
    if not s.isValid() or len(s.Solids())!=1:
        raise ValueError("invalid integral outer blank")
    return s

def finished_half(index, *, axis_z=None, reinforcement=0.):
    from modular_arm.v4 import joint
    b=outer_blank(index,axis_z=axis_z,reinforcement=reinforcement)
    return joint.fixed(b) if index==1 else joint.moving(b)
