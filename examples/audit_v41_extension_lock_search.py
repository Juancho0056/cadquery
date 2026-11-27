import argparse
from modular_arm.v4.extension_lock_search import search
p=argparse.ArgumentParser()
p.add_argument("archive")
p.add_argument("--output",default="out/v41_extension_lock_search")
a=p.parse_args()
r=search(a.archive,a.output)
print("Lock candidate:",r["accepted_lock_candidate"])
print("Clear folded pairs:",[(x["index_a"],x["index_b"]) for x in r["folded"] if x["clear"]])
