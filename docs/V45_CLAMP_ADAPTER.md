# V4.5 clamp adapter — unloaded fit prototype

Preserve the user-supplied modular_clamp_v1 (1).3mf clamp and its screw. No third-party meshes committed. The cylindrical insertion is nominally 27.4 mm diameter, 14 mm long, with a 39 mm flange. The upper male D is derived from the canonical V4 connection dimensions. It mates with a female joint head, not directly with the free male end of a telescopic segment. M4 x 35 button head, nut and washers are nominal hardware per modular_connection.py.

Generate with:
```powershell
python examples/build_v45_clamp_adapter.py
```

Outputs: STEP, STL, 3MF in out/v45_clamp_adapter. No slicer profile. Existing joint and arm modules unchanged.

## Gates
- CAD validity and connected solid checked by build().
- Clamp insertion fit, mating interference and axial retention: NOT_RUN on repository integration.
- Positive rotation lock, pull-out retention and physical load tests: NOT_RUN.
- Do not use under load until the clamp-side retention and complete assembly are verified.
- A separate female adapter is required if mounting a free male telescopic segment directly.

## Indexed revision
The original clamp mesh shows a radial locking screw axis 9 mm below the socket mouth (original Y=45, mouth Y=54). Four conical seats at 90-degree spacing are added to the stem; use the original locking screw. This is an indexed positioning candidate, not a continuous swivel. A nominal 1 mm shoulder/neck fillet preserves the bearing face. The underside shoulder may still require supports. Local CadQuery check with canonical dimension constants: valid single solid, 48 mm height, M4 gauge passage unobstructed. Full repository execution, original screw seating, pull-out, angular retention and load tests remain NOT_RUN. Do not load until verified.

## Assembly review generator
Run from repository root with the original local archive:

```powershell
python examples/build_v45_clamp_assembly.py "C:\path\modular_clamp_v1 (1).3mf"
```

The generator reads original body and locking screw by object IDs and names, measures the socket section at source Y=53.9, registers the adapter at the mouth, and positions the canonical magnetic V4.5 joint using its B socket. The screw tip placement is a trial, not a verified thread engagement. Exports a reference-only 3MF and JSON intersection report. Requires trimesh and manifold3d for volume checks. No original geometry is committed. This is a static subset (body, locking screw, adapter, elbow), not the complete clamp jaw assembly. Full generator execution and all assembled collision results are NOT_RUN at publication. A PASS of sampled volume checks is not qualification of shoulder contact, extraction, rotation, hardware, or load. Do not print the assembly file.
