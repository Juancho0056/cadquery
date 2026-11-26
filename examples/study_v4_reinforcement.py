"""Compare structural sections and actual integrated operating poses."""
import argparse,json,math
from modular_arm.v4.structural_screen import screen
from modular_arm.v4.integrated_arm import integrated_half,axis_heights,audit,mesh_library,cad_mesh
from modular_arm.v4 import joint
p=argparse.ArgumentParser()
p.add_argument("archive")
p.add_argument("--offset",type=float,default=15.4)
p.add_argument("--reinforcements",nargs="+",type=float,default=[0.,4.,6.])
a=p.parse_args()
z=axis_heights(a.offset)
tm=mesh_library()
fixed=integrated_half(a.archive,1,axis_z=z[1])
baseline=screen(a.offset)
reports=[]
for lift in a.reinforcements:
    report=screen(a.offset,reinforcement=lift)
    base=baseline["halves"]["5"]["minimum_wy"]["wy_mm3"]
    current=report["halves"]["5"]["minimum_wy"]["wy_mm3"]
    moving=integrated_half(a.archive,5,axis_z=z[5],reinforcement=lift)
    poses=[]
    for angle in range(-180,181,15):
        b=moving.copy()
        b.apply_transform(tm.transformations.rotation_matrix(math.radians(angle),(0,0,1)))
        for travel in ([0.,7.] if angle%45==0 else [7.]):
            p={"a":fixed,"b":b,"button":cad_mesh(joint.button(travel))}
            p.update({n:cad_mesh(s) for n,s in joint.screws(angle).items()})
            poses.append({"angle":angle,"travel":travel,"interferences":audit(p)})
    reports.append({"offset_mm":a.offset,"reinforcement_mm":lift,
       "minimum_wy_b":current,"improvement_percent":100*(current/base-1),
       "goal_met":current>=2000,"sections":report,"poses":poses})
print(json.dumps(reports,indent=2))
