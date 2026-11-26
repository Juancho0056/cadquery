import argparse
from modular_arm.v4.integrated_arm import export
p=argparse.ArgumentParser()
p.add_argument("archive")
p.add_argument("--output",default="out/v4_integrated")
p.add_argument("--angle",type=float,default=0.)
p.add_argument("--travel",type=float,default=0.)
a=p.parse_args()
for name,mesh in export(a.archive,a.output,a.angle,a.travel).items():
    print(name,"closed:",mesh.is_watertight,"volume mm3:",mesh.volume)
print("Experimental geometry only; not load rated or extension certified")
