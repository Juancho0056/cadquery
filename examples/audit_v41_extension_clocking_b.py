import argparse
from modular_arm.v4.extension_clocking_b import scan
p=argparse.ArgumentParser()
p.add_argument("archive")
p.add_argument("--output",default="out/v41_extension_clocking_b")
a=p.parse_args()
scan(a.archive,a.output)
