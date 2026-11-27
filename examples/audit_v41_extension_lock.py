import argparse
from modular_arm.v4.extension_lock import scan
p=argparse.ArgumentParser()
p.add_argument("archive")
p.add_argument("--output",default="out/v41_extension_lock")
a=p.parse_args()
scan(a.archive,a.output)
