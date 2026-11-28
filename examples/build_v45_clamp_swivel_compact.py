import argparse,json
from modular_arm.v45.clamp_swivel_compact import export

if __name__ == "__main__":
    p=argparse.ArgumentParser()
    p.add_argument("--output",default="out/v45_clamp_swivel_compact")
    args=p.parse_args()
    r=export(args.output)
    print(json.dumps({"passed":r["passed"],"checks":len(r["checks"]),"failures":[c for c in r["checks"] if c["status"]!="PASS"],"status":r["status"]},indent=2))
    if not r["passed"]: raise SystemExit(1)
