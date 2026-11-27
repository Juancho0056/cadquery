import argparse
from modular_arm.v4.extension_lock_assembly import report
p=argparse.ArgumentParser()
p.add_argument("archive")
p.add_argument("--output",default="out/v41_extension_lock_assembly")
a=p.parse_args()
report(a.archive,a.output)
