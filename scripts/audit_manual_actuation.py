"""Report real interference volumes; fail on any overlap. Experimental gate."""
import json
from itertools import combinations
from modular_arm.assembly.manual_actuation import manual_parts
from modular_arm.mechanisms.selector_contract import Mode

if __name__=="__main__":
    results=[]
    for mode in Mode:
        parts=manual_parts(mode=mode)
        errors=[]
        for name,p in parts.items():
            if not p.isValid() or len(p.Solids())!=1:
                errors.append({"invalid":name})
        for (a,p),(b,q) in combinations(parts.items(),2):
            volume=p.intersect(q).Volume()
            if volume>1e-6:
                errors.append({"pair":[a,b],"volume_mm3":volume})
        results.append({"mode":mode.value,"errors":errors})
    print(json.dumps(results,indent=2))
    raise SystemExit(int(any(r["errors"] for r in results)))
