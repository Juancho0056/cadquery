# Compact V4.5 swivel (review only)

Alternative to clamp_swivel.py. Original clamp interface and canonical D+M4 retained. Circular body Ø55 mm, local lateral boss and pin project beyond Ø55. Height 60 mm including 14 mm clamp stem and 16 mm D tenon. Previous nominal height 69.4 mm. No columns. Journal Ø24 mm, radial clearance 0.3 mm; rotor Ø42 mm. Twelve positions every 30 degrees. Top annular cap uses three bayonet lugs and a removable antirotation key, inspired by V4.5 joint. Captive radial Ø5 mm indexing pin with Ø7.4 shoulder, 5.2 mm travel; removable transverse safety key in either position. No additional metal screw in the swivel itself. Original clamp fastener and D+M4 remain.

## Assembly candidate
Insert pin from inside empty housing, retract, insert rotor, lower cap in entry position, rotate 30 degrees, insert cap seam key, engage pin and its safety key. Keys are removable and not qualified against accidental loss. Hold unloaded arm before unlocking. Review-only.

## Run
```powershell
python examples/build_v45_clamp_swivel_compact.py
```
Outputs locked/released review STEP and 3MF plus JSON. Individual printable exports intentionally disabled until verified. Tests include sampled pair overlaps, pin insertion/travel, rotor insertion, cap insertion/turn, key insertion and nominal stop overlap. All tests NOT_RUN at publication. Continuous motion, clamp/elbow integration, actual assembly, printability, fatigue, creep, retention under vibration and structural capacity NOT_RUN. No claim of 1 kg capacity. Do not print the assembled reference or use under load.
