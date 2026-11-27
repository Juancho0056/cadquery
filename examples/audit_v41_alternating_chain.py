import argparse
from modular_arm.v4.alternating_chain import scan
p=argparse.ArgumentParser()
p.add_argument("archive")
p.add_argument("--output",default="out/v41_alternating_chain")
a=p.parse_args()
scan(a.archive,a.output)
