"""Exact rigid-group reuse within each sampled sweep; resumable diagnostic."""
import hashlib
import importlib.metadata
import json
import os
from itertools import combinations
from pathlib import Path
from time import perf_counter
from modular_arm.v4 import alternating_chain as a
from modular_arm.v4 import modular_chain as c
from modular_arm.v4.extension_lock_assembly import overlap

CASES=((6,6,6),(6,6,6,6),(6,0,12,6),(6,12,0,6))
LIMIT=.01
SCHEMA=2

def group(name,moving):
    kind,index,part=name.split("_",2)
    index=int(index)
    if kind=="joint":
        if index!=moving: return int(index>moving)
        if part in ("head_a","button") or part.startswith("a_"): return 0
        if part=="head_b" or part.startswith(("b_","retainer_")): return 1
    elif kind=="segment" and part in ("inner","outer","lock"):
        return int(index>moving)
    raise ValueError("unknown component "+name)

class RigidAudit:
    """All pairs at zero; only cross-group pairs change during this sweep."""
    def __init__(self,parts,moving):
        self.names=tuple(parts)
        self.fixed=[]
        self.cross=[]
        self.calls=0
        for x,y in combinations(self.names,2):
            if group(x,moving)==group(y,moving):
                v=self.measure(parts,x,y)
                if v>LIMIT: self.fixed.append((x,y,v))
            else: self.cross.append((x,y))
    def measure(self,parts,x,y):
        self.calls+=1
        return overlap(parts[x],parts[y])
    def audit(self,parts):
        if tuple(parts)!=self.names: raise ValueError("assembly topology changed")
        defects=list(self.fixed)
        for x,y in self.cross:
            v=self.measure(parts,x,y)
            if v>LIMIT: defects.append((x,y,v))
        return defects

def digest(path):
    h=hashlib.sha256()
    with open(path,"rb") as stream:
        for block in iter(lambda:stream.read(1024*1024),b""): h.update(block)
    return h.hexdigest()

def identity(archive,mode):
    root=Path(__file__).resolve().parents[1]
    return {"schema":SCHEMA,"archive_sha256":digest(archive),"mode":mode,
            "source":{str(p.relative_to(root)):digest(p) for p in sorted(root.rglob("*.py"))},
            "versions":{n:importlib.metadata.version(n) for n in ("numpy","cadquery","trimesh","manifold3d")}}

def save(path,result):
    tmp=path.with_suffix(".tmp")
    with tmp.open("w",encoding="utf-8") as f:
        json.dump(result,f,indent=2,allow_nan=False)
        f.flush(); os.fsync(f.fileno())
    os.replace(tmp,path)

def jobs(mode):
    cases=CASES[:1] if mode=="quick" else CASES
    return [(indices,moving,sign) for indices in cases for moving in range(len(indices)-1) for sign in (-1,1)]

def sweep(archive,destination,*,mode="full",resume=False):
    if mode not in ("quick","full"): raise ValueError("invalid mode")
    dest=Path(destination); dest.mkdir(parents=True,exist_ok=True)
    path=dest/"report.json"
    ident=identity(archive,mode)
    tasks=jobs(mode)
    result={"status":"sampled_motion_only","complete":False,"identity":ident,"rows":[],
            "limitations":"15 degree samples; one joint at a time from straight. Not continuous clearance, safe angle limit, physical stop or load rating."}
    if resume:
        if not path.exists(): raise ValueError("no checkpoint to resume")
        result=json.loads(path.read_text(encoding="utf-8"))
        if result.get("identity")!=ident: raise ValueError("checkpoint identity mismatch; start fresh or use another output")
        if result.get("complete"): return result
    elif path.exists():
        # Preserve prior report unless explicitly starting a new output directory.
        raise ValueError("report already exists; use --resume or a new --output")
    save(path,result)
    source=a.templates(archive)
    meshes=a.JointMeshes()
    started=perf_counter()
    for task,(indices,moving,sign) in enumerate(tasks):
        rows=result["rows"]
        if task<len(rows):
            row=rows[task]
            if row.get("complete"): continue
        else:
            row={"indices":list(indices),"moving_joint":moving,"direction":sign,"samples":[],"complete":False}
            rows.append(row)
        n=len(indices)-1
        travels=[0.]*n; travels[moving]=7.
        def pose(magnitude):
            angles=[0.]*n; angles[moving]=float(sign*magnitude)
            return a.assemble(archive,angles,indices,source=source,travels=travels,joint_meshes=meshes)[0]
        baseline=pose(0)
        audit=RigidAudit(baseline,moving)
        for magnitude in range(0,181,15):
            if magnitude//15<len(row["samples"]): continue
            tick=perf_counter()
            parts=baseline if magnitude==0 else pose(magnitude)
            defects=audit.audit(parts)
            sample={"angle_deg":float(sign*magnitude),"collisions":defects,"clear":not defects}
            row["samples"].append(sample)
            clear=[x["angle_deg"] for x in row["samples"] if x["clear"]]
            row["last_clear_sample_deg"]=clear[-1] if clear else None
            row["first_collision_sample_deg"]=sample["angle_deg"] if defects else None
            row["complete"]=bool(defects) or magnitude==180
            save(path,result)
            print(f"[{task+1}/{len(tasks)}] angle={sample['angle_deg']:+.0f} clear={sample['clear']} sample={perf_counter()-tick:.2f}s elapsed={perf_counter()-started:.1f}s",flush=True)
            if row["complete"]: break
    result["complete"]=True
    save(path,result)
    return result
