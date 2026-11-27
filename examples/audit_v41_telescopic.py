import argparse,json
from pathlib import Path
from modular_arm.v4.telescopic_screen import report
p=argparse.ArgumentParser()
p.add_argument("archive")
p.add_argument("--output",default="out/v41_telescopic_screen.json")
a=p.parse_args()
r=report(a.archive)
dest=Path(a.output)
dest.parent.mkdir(parents=True,exist_ok=True)
dest.write_text(json.dumps(r,indent=2),encoding="utf-8")
print(json.dumps(r,indent=2))
