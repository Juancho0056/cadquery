# Compact V4.5 swivel (review only)

Alternative to clamp_swivel.py. Original clamp interface and canonical D+M4 retained. Circular body Ø55 mm, local lateral boss and pin project beyond Ø55. Height 60 mm including 14 mm clamp stem and 16 mm D tenon. Previous nominal height 69.4 mm. No columns. Journal Ø24 mm, radial clearance 0.3 mm; rotor Ø42 mm. Twelve positions every 30 degrees. Top annular cap uses three bayonet lugs and a removable antirotation key, inspired by V4.5 joint. Captive radial Ø5 mm indexing pin with Ø7.4 shoulder, 5.2 mm travel; removable transverse safety key in either position. No additional metal screw in the swivel itself. Original clamp fastener and D+M4 remain.

## Assembly candidate
Insert pin from inside empty housing, retract, insert rotor, lower cap in entry position, rotate 30 degrees, insert cap seam key, engage pin and its safety key. Keys are removable and not qualified against accidental loss. Hold unloaded arm before unlocking. Review-only.

## Run
```powershell
python examples/build_v45_clamp_swivel_compact.py
```
Outputs locked/released review STEP and 3MF plus JSON. Individual printable exports intentionally disabled until verified. Tests include sampled pair overlaps, pin insertion/travel, rotor insertion, cap insertion/turn, key insertion and nominal stop overlap. All tests NOT_RUN at publication. Continuous motion, clamp/elbow integration, actual assembly, printability, fatigue, creep, retention under vibration and structural capacity NOT_RUN. No claim of 1 kg capacity. Do not print the assembled reference or use under load.

## Cap revision: elbow-style closure
Added twelve peripheral scallops (radius 1.5 mm, same nominal radius as elbow) in the top flange. Retained three bayonet lugs and 30-degree rotation. The closure uses a manually inserted rigid seam key, not an automatic flexible detent; the key remains removable. Added sampled checks for reverse/forward key stop, housing reaction, bayonet end stop, extraction in unlocked position and rejection of incomplete closure. Additional 3MF review stages: CAP_ENTRY, CAP_INSERTED, CAP_SECURED. No change to rotor, lower clamp interface, D+M4 or radial indexing pin. All new checks NOT_RUN at publication. Review-only exports; no fabrication or load approval.

## User execution evidence — 2026-11-27
Supersedes earlier cap-key descriptions: cap_key and its seam were removed in aad5038; cap retains three bayonet lugs, without positive reverse stop. User supplied report.json: 303/303 sampled checks PASS, 0 FAIL. 240 pose pair checks (24 angles), 26 pin insertion, 6 rotor insertion, 5 pin_key insertion, 5 cap insertion, 7 cap turn, 6 pin travel and 8 retention/extraction checks. Status UNLOADED_REVIEW_ONLY. Report source commit not embedded; attribution to exact revision is not verified. Continuous assembly path, cap reverse retention, vibration, full clamp/joint integration and load remain NOT_RUN. Do not approve loaded use or interpret simulated overlap volume as strength. Next gate: integration with original clamp and elbow, then unloaded physical fit prototype.

## Integration review
Run `python examples/build_v45_clamp_swivel_integration.py "C:\path\modular_clamp_v1 (1).3mf"`. Uses unchanged compact swivel, magnetic V4.5 elbow and canonical B hardware envelopes; original clamp body and trial attachment screw read from local archive. Samples 24 angles at 15-degree intervals. Cross-module overlap checks and five review scenes. Report records Git HEAD, dirty flag and input SHA256. Trial screw pose does not prove thread fit. Original lower jaw and main clamping screw are not included. Internal swivel tests remain separately reported. Integration execution NOT_RUN at publication; individual fabrication exports remain disabled. No load qualification.
