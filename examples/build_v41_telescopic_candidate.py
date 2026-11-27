import argparse
from modular_arm.v4.telescopic_candidate import scan
p=argparse.ArgumentParser()
p.add_argument("archive")
p.add_argument("--output",default="out/v41_telescopic_candidate")
p.add_argument("--quick",action="store_true",help="Test only zero shaft clocking")
a=p.parse_args()
r=scan(a.archive,a.output,**({"angles":(0.,)} if a.quick else {}))
print("Deepest clear sampled pose:",r["deepest_sample_clear"])
print("Report:",a.output+"/report.json")
