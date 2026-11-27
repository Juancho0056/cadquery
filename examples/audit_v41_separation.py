import argparse
from modular_arm.v4.separation_variant import scan
p=argparse.ArgumentParser()
p.add_argument("archive")
p.add_argument("--output",default="out/v41_separation")
a=p.parse_args()
r=scan(a.archive,a.output)
print("Selected delta:",r["selected_delta_mm"])
