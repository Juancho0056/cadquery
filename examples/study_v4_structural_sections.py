"""Print geometric neck screening results; no FEA."""
import argparse,json
from modular_arm.v4.structural_screen import screen
p=argparse.ArgumentParser()
p.add_argument("--offsets",nargs="+",type=float,default=[15.4,10.,5.,0.])
a=p.parse_args()
for offset in a.offsets:
    print(json.dumps(screen(offset),indent=2),flush=True)
