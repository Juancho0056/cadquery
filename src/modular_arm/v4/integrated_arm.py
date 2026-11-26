"""Replace original terminal geometry with integral V4 joint halves.
Local-only derived geometry. Requires optional reference mesh boolean backend.
"""
import math
import json
from pathlib import Path
import numpy as np
import cadquery as cq
from modular_arm.v4 import joint
from modular_arm.v4.reference_arm import read_part

CUTS={1:105.,5:40.}
CENTERS={1:(105.62144,153.20654),5:(146.39,151.505)}
JOIN_X=46.
CENTER_Z={1:4.,5:19.4}

def mesh_library():
    import trimesh
    import manifold3d  # noqa: F401
    return trimesh

def require_solid(m,label):
    if m.is_empty:
        raise ValueError(f"{label}: empty mesh")
    components=m.split(only_watertight=False)
    if not m.is_volume or len(components)!=1:
        raise ValueError(f"{label}: expected one closed positive-volume body; "
                         f"watertight={m.is_watertight}, winding={m.is_winding_consistent}, "
                         f"volume={m.volume}, components={len(components)}, "
                         f"component_volumes={[float(c.volume) for c in components]}")
    return m

def cad_mesh(shape):
    tm=mesh_library()
    vertices,faces=shape.tessellate(.05,.08)
    m=tm.Trimesh(vertices=[v.toTuple() for v in vertices],faces=faces,process=True)
    m.fix_normals()
    return require_solid(m,"CAD")

def original_mesh(archive,index):
    tm=mesh_library()
    t=read_part(archive,index).triangles.astype(float)
    m=tm.Trimesh(vertices=t.reshape(-1,3),faces=np.arange(t.size//3).reshape(-1,3),process=True)
    m.fix_normals()
    return require_solid(m,f"obj_{index}")

def crop_terminal(m,index):
    """Keep only original longitudinal body: no original pin hole or fork."""
    tm=mesh_library()
    lo,hi=m.bounds.copy()
    lo-=1
    hi+=1
    if index==1:
        hi[2]=CUTS[index]
    elif index==5:
        lo[2]=CUTS[index]
    else:
        raise ValueError("unsupported reference body")
    b=tm.creation.box(extents=hi-lo,transform=tm.transformations.translation_matrix((lo+hi)/2))
    return require_solid(tm.boolean.intersection([m,b],engine="manifold"),"cropped original")

def align_body(m,index,*,axis_z=None):
    m=m.copy()
    cx,cy=CENTERS[index]
    m.apply_translation((-cx,-cy,-CUTS[index]))
    # Original +Z maps to +X, transverse X maps to -Z.
    m.apply_transform(np.array([[0,0,1,0],[0,1,0,0],[-1,0,0,0],[0,0,0,1]],dtype=float))
    m.apply_translation((-JOIN_X if index==1 else JOIN_X,0,CENTER_Z[index] if axis_z is None else axis_z))
    return m

def joint_half(index,*,axis_z=None,reinforcement=0.,straight=False):
    """Construct exterior first; reuse the canonical V4 machining operations."""
    from modular_arm.v4.transition import finished_half
    if straight:
        from modular_arm.v4.straight_arm import finished_straight
        result=finished_straight(index)
    else:
        result=finished_half(index,axis_z=axis_z,reinforcement=reinforcement)
    if not result.isValid() or len(result.Solids())!=1:
        raise ValueError("invalid integrated CAD half")
    return result

def joining_half(index,*,axis_z=None,reinforcement=0.,straight=False):
    """Keep exterior up to the cut; use a buried spigot to avoid near-coincident skins."""
    s=joint_half(index,axis_z=axis_z,reinforcement=reinforcement,straight=straight)
    direction=-1 if index==1 else 1
    # The measured original section is solid near the cut. The smaller
    # spigot deliberately stays well inside that section, not on its skin.
    start=direction*(JOIN_X-.5)
    cut=cq.Solid.makeBox(100,200,200,cq.Vector(JOIN_X if index==5 else -JOIN_X-100,-100,-100))
    s=s.cut(cut)
    spigot=cq.Solid.makeCylinder(11.,2.,cq.Vector(start,0,CENTER_Z[index] if axis_z is None else axis_z),cq.Vector(direction,0,0))
    s=s.fuse(spigot).clean()
    if not s.isValid() or len(s.Solids())!=1:
        raise ValueError("invalid joining half")
    return s

def protected_region(mesh,index):
    """Longitudinal material beyond the explicitly allowed 2 mm join zone."""
    tm=mesh_library()
    lo,hi=mesh.bounds.copy()
    lo-=1
    hi+=1
    if index==1:
        hi[0]=-JOIN_X-2.
    else:
        lo[0]=JOIN_X+2.
    return tm.creation.box(extents=hi-lo,transform=tm.transformations.translation_matrix((lo+hi)/2))

def preservation_report(original,integrated,index):
    tm=mesh_library()
    region=protected_region(original,index)
    a=tm.boolean.intersection([original,region],engine="manifold")
    b=tm.boolean.intersection([integrated,region],engine="manifold")
    if a.is_empty or b.is_empty:
        raise ValueError("protected body unexpectedly empty")
    added=tm.boolean.difference([b,a],engine="manifold")
    removed=tm.boolean.difference([a,b],engine="manifold")
    return {"added_mm3":0. if added.is_empty else abs(float(added.volume)),
            "removed_mm3":0. if removed.is_empty else abs(float(removed.volume))}

def integrated_half(archive,index,*,axis_z=None,reinforcement=0.,straight=False):
    """Boolean fuse, never concatenate disconnected shells or silently fall back."""
    tm=mesh_library()
    body=align_body(crop_terminal(original_mesh(archive,index),index),index,axis_z=axis_z)
    half=cad_mesh(joining_half(index,axis_z=axis_z,reinforcement=reinforcement,straight=straight))
    overlap=tm.boolean.intersection([body,half],engine="manifold")
    if overlap.is_empty or overlap.volume<=1:
        raise ValueError("insufficient overlap for integral union")
    # Keep exact manifold topology from the boolean; do not discard small
    # components or repair by filling holes, which could alter the original.
    raw=tm.boolean.union([body,half],engine="manifold")
    try:
        result=require_solid(raw,f"integrated_{index}")
    except ValueError as exc:
        raise ValueError(f"{exc}; overlap_mm3={overlap.volume:.6f}") from exc
    report=preservation_report(body,result,index)
    if max(report.values())>.05:
        raise ValueError(f"original longitudinal body modified: {report}")
    return result

def axis_heights(offset=15.4):
    if not math.isfinite(offset) or not 0<=offset<=15.4:
        raise ValueError("offset must be between 0 and 15.4 mm")
    if offset == 15.4:
        return dict(CENTER_Z)
    return {1:11.7-offset/2,5:11.7+offset/2}

def parts(archive,angle=0.,travel=0.,*,offset=15.4,reinforcement=0.,straight=False):
    if not math.isfinite(angle):
        raise ValueError("invalid angle")
    # Retain original indexing and travel contract.
    base=joint.parts(angle,travel)
    tm=mesh_library()
    if straight:
        from modular_arm.v4.straight_arm import AXES
        if reinforcement or offset!=15.4:
            raise ValueError('straight profile has fixed axes; omit offset and reinforcement')
        heights=AXES
    else:
        heights=axis_heights(offset)
    a=integrated_half(archive,1,axis_z=heights[1],straight=straight)
    b=integrated_half(archive,5,axis_z=heights[5],reinforcement=reinforcement,straight=straight)
    b.apply_transform(tm.transformations.rotation_matrix(math.radians(angle),(0,0,1)))
    return {"arm_a_integrated":a,"arm_b_integrated":b,
            **{n:cad_mesh(s) for n,s in base.items() if n not in ("arm_a","arm_b")}}

def intersection_volume(mesh):
    """Signed tetrahedral volume without computing center of mass.
    A zero-volume contact is not a solid penetration. Reject invalid nonzero
    results rather than masking them. Use a local origin for numerical stability.
    """
    if mesh.is_empty:
        return 0.
    t=np.asarray(mesh.triangles,dtype=np.float64)
    if not np.isfinite(t).all():
        raise ValueError("nonfinite intersection triangles")
    t=t-t[0,0]
    volume=float(np.einsum("ij,ij->i",t[:,0],np.cross(t[:,1],t[:,2])).sum()/6.)
    if not math.isfinite(volume):
        raise ValueError("nonfinite intersection volume")
    if abs(volume)<=1e-9:
        return 0.
    if volume<0 or not mesh.is_watertight or not mesh.is_winding_consistent:
        raise ValueError(f"invalid nonzero intersection: {volume} mm3")
    return volume

def audit(parts):
    from itertools import combinations
    tm=mesh_library()
    defects=[]
    for (a,s),(b,t) in combinations(parts.items(),2):
        if np.any(s.bounds[1]<t.bounds[0]) or np.any(t.bounds[1]<s.bounds[0]):
            continue
        q=tm.boolean.intersection([s,t],engine="manifold")
        volume=intersection_volume(q)
        if volume>.01:
            defects.append((a,b,volume))
    return defects

PRINTABLE=("arm_a_integrated","arm_b_integrated","button")

def export(archive,destination,angle=0.,travel=0.,*,offset=15.4,reinforcement=0.,straight=False):
    p=parts(archive,angle,travel,offset=offset,reinforcement=reinforcement,straight=straight)
    defects=audit(p)
    if defects:
        raise ValueError(f"interference mm3: {defects}")
    # Export neutral individual parts rather than the operating pose.
    neutral=parts(archive,0.,0.,offset=offset,reinforcement=reinforcement,straight=straight) if (angle,travel)!=(0.,0.) else p
    dest=Path(destination)
    printable=dest/"prototype_printable"
    assembly=dest/"assembly_preview"
    printable.mkdir(parents=True,exist_ok=True)
    assembly.mkdir(parents=True,exist_ok=True)
    for n,m in p.items():
        m.export(assembly/(n+".stl"))
    for n in PRINTABLE:
        m=neutral[n].copy()
        if straight and n != 'button':
            # Tube end down, joint up. Rotate longitudinal X into build Z.
            sign=-1 if n=='arm_a_integrated' else 1
            m.apply_transform(mesh_library().transformations.rotation_matrix(sign*math.pi/2,(0,1,0)))
        m.apply_translation(-m.bounds[0])
        m.export(printable/(n+".stl"))
    manifest={"status":"experimental_not_load_rated",
              "printable":list(PRINTABLE),
              "metal_hardware":[{"item":"radial retainer","quantity":3,"spec":"nominal M3 dog-point; exact length and thread pending"},
                                {"item":"compression spring","quantity":1,"spec":"OD 18 mm envelope; installed 13 mm, compressed 6 mm; commercial part pending"}],
              "assembly_pose":{"angle":angle,"travel":travel,"axis_offset_mm":29. if straight else offset,"profile":"straight" if straight else "curved","b_reinforcement_mm":reinforcement},
              "notes":"Only prototype_printable contains candidate printed parts. Assembly preview is not a print plate. Spring not exported as a solid. Printing orientation not qualified."}
    (dest/"manifest.json").write_text(json.dumps(manifest,indent=2),encoding="utf-8")
    return p
