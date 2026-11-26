"""Read-only inventory of 3MF model resources, not an engineering certification."""
from hashlib import sha256
from pathlib import Path
from zipfile import ZipFile
import xml.etree.ElementTree as ET
import math

CORE = "{http://schemas.microsoft.com/3dmanufacturing/core/2015/02}"
UNIT_MM = {"micron": .001, "millimeter": 1., "centimeter": 10.,
           "inch": 25.4, "foot": 304.8, "meter": 1000.}
MAX_MODEL_BYTES = 64 * 1024 * 1024


def inventory(path: str | Path) -> dict:
    path = Path(path)
    digest = sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    models = []
    with ZipFile(path) as archive:
        for entry in archive.infolist():
            if not entry.filename.lower().endswith(".model"):
                continue
            if entry.file_size > MAX_MODEL_BYTES:
                raise ValueError("model exceeds inspection size limit")
            data = archive.read(entry)
            if b"<!DOCTYPE" in data.upper() or b"<!ENTITY" in data.upper():
                raise ValueError("DTD/entity declarations not supported")
            root = ET.fromstring(data)
            if root.tag != CORE + "model":
                raise ValueError("unsupported 3MF model namespace")
            unit = root.get("unit", "millimeter")
            if unit not in UNIT_MM:
                raise ValueError(f"unsupported unit: {unit}")
            scale = UNIT_MM[unit]
            objects = []
            for obj in root.findall(f"{CORE}resources/{CORE}object"):
                vertices = obj.findall(f"{CORE}mesh/{CORE}vertices/{CORE}vertex")
                points = [tuple(float(v.attrib[a]) * scale for a in "xyz") for v in vertices]
                if any(not math.isfinite(n) for p in points for n in p):
                    raise ValueError("non-finite vertex")
                triangles = obj.findall(f"{CORE}mesh/{CORE}triangles/{CORE}triangle")
                for triangle in triangles:
                    indices = [int(triangle.attrib[k]) for k in ("v1", "v2", "v3")]
                    if any(i < 0 or i >= len(points) for i in indices):
                        raise ValueError("triangle references missing vertex")
                bounds = None
                if points:
                    bounds = {a: [min(p[i] for p in points), max(p[i] for p in points)]
                              for i, a in enumerate("xyz")}
                objects.append({"id": obj.attrib["id"], "name": obj.get("name"),
                                "vertices": len(points), "triangles": len(triangles),
                                "local_bounds_mm": bounds,
                                "components": [dict(c.attrib) for c in
                                               obj.findall(f"{CORE}components/{CORE}component")]})
            models.append({"entry": entry.filename, "unit": unit, "objects": objects,
                           "build_items": [dict(i.attrib) for i in
                                           root.findall(f"{CORE}build/{CORE}item")]})
    if not models:
        raise ValueError("no model entries found")
    return {"filename": path.name, "sha256": digest.hexdigest(), "models": models,
            "license_status": "NOT_RUN", "mechanical_validation": "NOT_RUN",
            "limitations": "Local bounds only; component/build transforms not resolved; no topology or mesh watertightness certification."}
