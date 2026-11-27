import argparse
from modular_arm.v4.chain_motion_screen import sweep
p=argparse.ArgumentParser()
p.add_argument("archive")
p.add_argument("--output",default="out/v41_chain_motion")
a=p.parse_args()
sweep(a.archive,a.output)
