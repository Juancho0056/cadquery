"""Geometric section comparison of the modular joint; not stress analysis.
All dimensions mm. Components reported independently, not as a bonded section.
"""
from dataclasses import asdict
import math
from modular_arm.v4 import modular_connection as m
from modular_arm.v4.structural_screen import measure

STATIONS=(30.5,32.,34.,35.5,36.,37.,38.,39.,40.,40.5,42.,44.,45.5)

def summary(rows):
    return {k:min(rows,key=lambda r:r[k]) for k in ("area_mm2","wy_mm3","wz_mm3")}

def report():
    result={}
    for i,label in ((1,"a"),(5,"b")):
        sign=m.direction(i)
        head=m.head(i)
        male=m.tenon(i)
        baseline=m.flush_arm.finished_flush(i)
        unbored=m.keyed(i,m.JOIN-m.LENGTH,m.JOIN+1.5)
        rows=[]
        for station in STATIONS:
            x=station*sign
            h=asdict(measure(head,x))
            t=asdict(measure(male,x))
            ref=asdict(measure(baseline,x))
            u=asdict(measure(unbored,x))
            rows.append({"station_mm":station,"head":h,"tenon":t,
                         "integral_reference":ref,"unbored_tenon":u,
                         "head_area_fraction":h["area_mm2"]/ref["area_mm2"],
                         "tenon_area_fraction":t["area_mm2"]/u["area_mm2"],
                         "tenon_wy_fraction":t["wy_mm3"]/u["wy_mm3"],
                         "tenon_wz_fraction":t["wz_mm3"]/u["wz_mm3"]})
        # Do not sum section moduli across sliding interface.
        result[label]={"sections":rows,"head_minima":summary([r["head"] for r in rows]),
                       "tenon_minima":summary([r["tenon"] for r in rows]),
                       "nominal_center_wall_mm":m.SEAT_Y-m.RADIUS-m.CLEARANCE,
                       "axial_ligament_to_tenon_end_mm":m.PIN_AT-(m.JOIN-m.LENGTH)-m.PIN_RADIUS,
                       "axial_ligament_to_root_mm":m.JOIN-m.PIN_AT-m.PIN_RADIUS}
    return {"status":"geometric_screen_not_load_rated","halves":result,
            "limitations":"Sampled planar sections, excludes real STL overlap/root, stress concentration, layer anisotropy, fastener shear, bearing, contact and preload."}
