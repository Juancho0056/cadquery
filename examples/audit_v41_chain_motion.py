import argparse
from modular_arm.v4.chain_motion_screen import sweep
p=argparse.ArgumentParser()
p.add_argument("archive")
p.add_argument("--output",default="out/v41_chain_motion_optimized")
p.add_argument("--mode",choices=("quick","full"),default="full")
p.add_argument("--resume",action="store_true")
a=p.parse_args()
sweep(a.archive,a.output,mode=a.mode,resume=a.resume)
