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
