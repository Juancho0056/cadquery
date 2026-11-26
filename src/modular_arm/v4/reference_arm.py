"""Read exact original STL geometry locally without vendoring licensed assets."""
from dataclasses import dataclass
from pathlib import Path
import re
import struct
import zipfile
import numpy as np

@dataclass(frozen=True)
class ReferencePart:
    name: str
    triangles: np.ndarray

    @property
    def bounds(self):
        return self.triangles.min(axis=(0,1)),self.triangles.max(axis=(0,1))

    def section(self,z):
        edges=[]
        for j in range(3):
            p=self.triangles[:,j]
            q=self.triangles[:,(j+1)%3]
            mask=(p[:,2]-z)*(q[:,2]-z)<0
            p,q=p[mask],q[mask]
            t=(z-p[:,2])/(q[:,2]-p[:,2])
            edges.append(p+(q-p)*t[:,None])
        return np.concatenate(edges)

def read_part(archive,index):
    if not isinstance(index,int) or not 1<=index<=40:
        raise ValueError("invalid reference part index")
    with zipfile.ZipFile(Path(archive)) as z:
        names=[n for n in z.namelist() if re.fullmatch(fr"obj_{index}_[^/]+\.stl",n)]
        if len(names)!=1:
            raise ValueError("reference part missing or ambiguous")
        raw=z.read(names[0])
    count=struct.unpack_from("<I",raw,80)[0]
    if len(raw)!=84+50*count:
        raise ValueError("invalid binary STL")
    dtype=np.dtype([("n","<f4",3),("v","<f4",(3,3)),("a","<u2")])
    return ReferencePart(names[0],np.frombuffer(raw,dtype,offset=84)["v"])

def journal_report(archive):
    p=read_part(archive,3)
    result=[]
    for h in (20.1,25.1,30.1,34.9):
        xy=p.section(h)[:,:2]
        center=(xy.min(0)+xy.max(0))/2
        r=np.linalg.norm(xy-center,axis=1)
        result.append({"z":h,"center":center.tolist(),"diameter_min":float(2*r.min()),"diameter_max":float(2*r.max())})
    return result
