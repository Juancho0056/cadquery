"""Run with python examples/inspect_v4_reference_arm.py PATH_TO_ORIGINAL_ZIP."""
import argparse
import json
from modular_arm.v4.reference_arm import read_part,journal_report
parser=argparse.ArgumentParser()
parser.add_argument("archive")
args=parser.parse_args()
for i in (1,3,5):
    p=read_part(args.archive,i)
    lo,hi=p.bounds
    print(p.name,"dimensions mm:",(hi-lo).tolist())
print(json.dumps(journal_report(args.archive),indent=2))
