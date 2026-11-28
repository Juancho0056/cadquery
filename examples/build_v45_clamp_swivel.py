import argparse,json
from modular_arm.v45.clamp_swivel import export

if __name__ == '__main__':
    p=argparse.ArgumentParser()
    p.add_argument('--output',default='out/v45_clamp_swivel')
    args=p.parse_args()
    report=export(args.output)
    print(json.dumps({k:v for k,v in report.items() if k!='sampled_checks'},indent=2))
    if not report['passed']: raise SystemExit(1)
