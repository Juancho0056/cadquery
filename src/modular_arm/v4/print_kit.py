"""Unloaded fit prototype kit for the locally screened 29.4 mm joint."""
import json,math
from pathlib import Path
from modular_arm.v4 import separation_variant as s
from modular_arm.v4 import spaced_chain as chain
ia,m=s.ia,s.m

def quantities(segments):
    if not isinstance(segments,int) or segments<2: raise ValueError("at least two segments")
    j=segments-1
    return {"head_a":j,"head_b":j,"button":j,"tube_inner_standard":1,
            "tube_inner_reclocked":j,"tube_outer":segments,"extension_screw":segments}

def export(archive,destination,segments=2):
    counts=quantities(segments)
    builder=chain.Builder(archive)
    inner,outer,screw,flipped=builder.source
    # Audit exactly the requested straight chain before producing a kit.
    parts,_=builder.parts((0.,)*(segments-1),(12,)*segments)
    defects=ia.audit(parts)
    if defects: raise ValueError(f"straight assembly interference: {defects}")
    if segments==2:
        folded,_=builder.parts((180.,),(12,12))
        defects=ia.audit(folded)
        if defects: raise ValueError(f"folded assembly interference: {defects}")
    else: folded=None
    meshes={"head_a":builder.heads[1],"head_b":builder.heads[5],
            "button":builder.buttons[0.],"tube_inner_standard":inner,
            "tube_inner_reclocked":flipped,"tube_outer":outer,"extension_screw":screw}
    # Same vertical tube orientation as canonical modular export.
    # Original adjustment screw retains original print axis.
    rotations={"head_a":90,"head_b":-90,"tube_inner_standard":-90,
               "tube_inner_reclocked":-90,"tube_outer":90}
    dest=Path(destination); dest.mkdir(parents=True,exist_ok=True)
    printable=dest/"PRINT_STL"; printable.mkdir(exist_ok=True)
    for old in printable.glob("*.stl"): old.unlink()
    tm=ia.mesh_library()
    records=[]
    for name,mesh in meshes.items():
        body=ia.require_solid(mesh.copy(),name)
        if name in rotations:
            body.apply_transform(tm.transformations.rotation_matrix(math.radians(rotations[name]),(0,1,0)))
        body.apply_translation(-body.bounds[0])
        body.export(printable/(name+".stl"))
        records.append({"file":name+".stl","quantity":counts[name],
                        "dimensions_mm":body.extents.tolist()})
    for name,assembly in (("PREVIEW_STRAIGHT",parts),("PREVIEW_FOLDED",folded)):
        folder=dest/name
        if folder.exists():
            for old in folder.glob("*.stl"): old.unlink()
        if assembly is not None:
            folder.mkdir(exist_ok=True)
            for label,mesh in assembly.items(): mesh.export(folder/(label+".stl"))
    j=segments-1
    manifest={"status":"unloaded_fit_prototype_only","axis_spacing_mm":29.4,
              "segments":segments,"joints":j,"printable":records,
              "metal_hardware":[{"item":"M4 x 35 button head screw","quantity":2*j},
              {"item":"M4 prevailing torque nut","quantity":2*j},
              {"item":"M4 washer","quantity":4*j},
              {"item":"nominal M3 dog point retainer (verify length/tip)","quantity":3*j},
              {"item":"compression spring (verify actual fit; envelope OD18, free23, solid <=5 mm)","quantity":j}],
              "notes":"Print only PRINT_STL. Previews include metal envelopes. No base or gripper. Candidate orientation, inspect supports and bed adhesion. No physical strength, continuous motion or thread qualification."}
    (dest/"manifest.json").write_text(json.dumps(manifest,indent=2),encoding="utf-8")
    lines=["# V4.1 — Prototipo modular 29,4 mm","","Imprimir únicamente PRINT_STL. Cantidades por archivo:",""]
    lines.extend(f"- {x['file']}: {x['quantity']}" for x in records)
    lines.extend(["","## Herrajes (dimensiones nominales)"])
    lines.extend(f"- {x['quantity']} × {x['item']}" for x in manifest["metal_hardware"])
    lines.extend(["","PETG como candidato: boquilla 0,4 mm, capa 0,2 mm, 6 perímetros, relleno gyroid 50%, brim 12 mm para tubos. Revisar soportes en laminador; no bloquear cavidades ni roscas.","Montar sin carga, verificar ajuste y bloqueo. No usar sobre personas ni objetos valiosos. No certificado para carga."])
    (dest/"LEEME.md").write_text("\n".join(lines)+"\n",encoding="utf-8")
    return manifest
