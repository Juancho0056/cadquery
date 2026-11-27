import argparse
from modular_arm.v4.extension_pair import scan
p=argparse.ArgumentParser()
p.add_argument("archive")
p.add_argument("--output",default="out/v41_extension_pair")
p.add_argument("--quick",action="store_true",help="Only relative clock 0, three insertion depths")
a=p.parse_args()
r=scan(a.archive,a.output,**({"depths":(35.,65.,95.),"clocks":(0.,)} if a.quick else {}))
print("Selected pair:",r["selected_pair"])
