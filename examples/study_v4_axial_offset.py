"""Compare actual integrated mesh poses; report collision, never auto-certify."""
import argparse,json
from modular_arm.v4.integrated_arm import integrated_half,axis_heights,audit,mesh_library,cad_mesh
from modular_arm.v4 import joint
import math
p=argparse.ArgumentParser()
p.add_argument("archive")
p.add_argument("--offsets",nargs="+",type=float,default=[15.4,10.,5.,0.])
p.add_argument("--step",type=int,default=15)
a=p.parse_args()
if not 1<=a.step<=45:
    p.error("step must be 1..45")
tm=mesh_library()
for offset in a.offsets:
    z=axis_heights(offset)
    fixed=integrated_half(a.archive,1,axis_z=z[1])
    moving=integrated_half(a.archive,5,axis_z=z[5])
    rows=[]
    for angle in sorted(set(range(-180,181,a.step))|{0,180,-180,45,-45,90,-90,135,-135}):
        b=moving.copy()
        b.apply_transform(tm.transformations.rotation_matrix(math.radians(angle),(0,0,1)))
        solids={"arm_a":fixed,"arm_b":b,"button":cad_mesh(joint.button(7.))}
        solids.update({n:cad_mesh(s) for n,s in joint.screws(angle).items()})
        defects=audit(solids)
        rows.append({"angle":angle,"interferences":defects})
    print(json.dumps({"offset_mm":offset,"axis_z":z,"travel_mm":7,"sampled_poses":rows},indent=2),flush=True)
