import argparse
from modular_arm.v4.extension_clocking import scan
p=argparse.ArgumentParser()
p.add_argument("archive")
p.add_argument("--output",default="out/v41_extension_clocking")
a=p.parse_args()
r=scan(a.archive,a.output)
for clock in (0,180):
    rows=[x for x in r["rows"] if x["clock_a_deg"]==clock and x["angle_deg"]==180]
    print("Clock A",clock,"clear folded",sum(x["clear"] for x in rows),"/",len(rows))
