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
    if m.is_empty or not m.is_volume or len(m.split(only_watertight=False))!=1:
        raise ValueError(f"{label}: expected one closed positive-volume body")
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

def align_body(m,index):
    m=m.copy()
    cx,cy=CENTERS[index]
    m.apply_translation((-cx,-cy,-CUTS[index]))
    # Original +Z maps to +X, transverse X maps to -Z.
    m.apply_transform(np.array([[0,0,1,0],[0,1,0,0],[-1,0,0,0],[0,0,0,1]],dtype=float))
    m.apply_translation((-JOIN_X if index==1 else JOIN_X,0,CENTER_Z[index]))
    return m

def joint_half(index):
    """Retain the core and replace the narrow terminal with a progressive loft."""
    if index not in CUTS:
        raise ValueError("unsupported reference body")
    s=joint.fixed() if index==1 else joint.moving()
    direction=-1 if index==1 else 1
    z=CENTER_Z[index]
    # Remove the old rounded neck tip, which otherwise protrudes through
    # the new transition. Preserve all core features within radius 28.
    clip=cq.Solid.makeBox(100,200,200,cq.Vector(39 if index==5 else -139,-100,-100))
    s=s.cut(clip)
    # Four smooth oval sections, with a constant-sized overlap at the end.
    # Larger minimum section than the previous 13 x 6.5 mm semiaxes.
    sections=((30,16.0,7.0),(36,15.5,9.0),(42,14.2,12.0),
              (46,13.7,13.3),(48,13.7,13.3))
    wires=[cq.Workplane("YZ",origin=(direction*x,0,z)).ellipse(y,h).val()
           for x,y,h in sections]
    bridge=cq.Solid.makeLoft(wires,ruled=False)
    result=s.fuse(bridge).clean()
    if not result.isValid() or len(result.Solids())!=1:
        raise ValueError("invalid reinforced transition")
    return result

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

def integrated_half(archive,index):
    """Boolean fuse, never concatenate disconnected shells or silently fall back."""
    tm=mesh_library()
    body=align_body(crop_terminal(original_mesh(archive,index),index),index)
    half=cad_mesh(joint_half(index))
    overlap=tm.boolean.intersection([body,half],engine="manifold")
    if overlap.is_empty or overlap.volume<=1:
        raise ValueError("insufficient overlap for integral union")
    result=require_solid(tm.boolean.union([body,half],engine="manifold"),f"integrated_{index}")
    report=preservation_report(body,result,index)
    if max(report.values())>.05:
        raise ValueError(f"original longitudinal body modified: {report}")
    return result

def parts(archive,angle=0.,travel=0.):
    if not math.isfinite(angle):
        raise ValueError("invalid angle")
    # Retain original indexing and travel contract.
    base=joint.parts(angle,travel)
    tm=mesh_library()
    a=integrated_half(archive,1)
    b=integrated_half(archive,5)
    b.apply_transform(tm.transformations.rotation_matrix(math.radians(angle),(0,0,1)))
    return {"arm_a_integrated":a,"arm_b_integrated":b,
            **{n:cad_mesh(s) for n,s in base.items() if n not in ("arm_a","arm_b")}}

def audit(parts):
    from itertools import combinations
    tm=mesh_library()
    defects=[]
    for (a,s),(b,t) in combinations(parts.items(),2):
        if np.any(s.bounds[1]<t.bounds[0]) or np.any(t.bounds[1]<s.bounds[0]):
            continue
        q=tm.boolean.intersection([s,t],engine="manifold")
        if not q.is_empty and q.volume>.01:
            defects.append((a,b,float(q.volume)))
    return defects

PRINTABLE=("arm_a_integrated","arm_b_integrated","button")

def export(archive,destination,angle=0.,travel=0.):
    p=parts(archive,angle,travel)
    defects=audit(p)
    if defects:
        raise ValueError(f"interference mm3: {defects}")
    # Export neutral individual parts rather than the operating pose.
    neutral=parts(archive,0.,0.) if (angle,travel)!=(0.,0.) else p
    dest=Path(destination)
    printable=dest/"prototype_printable"
    assembly=dest/"assembly_preview"
    printable.mkdir(parents=True,exist_ok=True)
    assembly.mkdir(parents=True,exist_ok=True)
    for n,m in p.items():
        m.export(assembly/(n+".stl"))
    for n in PRINTABLE:
        m=neutral[n].copy()
        m.apply_translation(-m.bounds[0])
        m.export(printable/(n+".stl"))
    manifest={"status":"experimental_not_load_rated",
              "printable":list(PRINTABLE),
              "metal_hardware":[{"item":"radial retainer","quantity":3,"spec":"nominal M3 dog-point; exact length and thread pending"},
                                {"item":"compression spring","quantity":1,"spec":"OD 18 mm envelope; installed 13 mm, compressed 6 mm; commercial part pending"}],
              "assembly_pose":{"angle":angle,"travel":travel},
              "notes":"Only prototype_printable contains candidate printed parts. Assembly preview is not a print plate. Spring not exported as a solid. Printing orientation not qualified."}
    (dest/"manifest.json").write_text(json.dumps(manifest,indent=2),encoding="utf-8")
    return p
