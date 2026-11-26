"""Report all rigid overlaps in the integrated selector proposal."""
import json
from itertools import combinations
from modular_arm.assembly.common_selector import common_selector_parts
from modular_arm.mechanisms.selector_contract import Mode

if __name__=="__main__":
    report=[]
    for mode in Mode:
        parts=common_selector_parts(mode=mode,include_arms=True)
        defects=[]
        for name,p in parts.items():
            if not p.isValid() or len(p.Solids())!=1:
                defects.append({"invalid":name})
        for (a,p),(b,q) in combinations(parts.items(),2):
            volume=p.intersect(q).Volume()
            if volume>1e-6:
                defects.append({"pair":[a,b],"volume_mm3":volume})
        report.append({"mode":mode.value,"defects":defects})
    print(json.dumps(report,indent=2))
    raise SystemExit(int(any(x["defects"] for x in report)))
