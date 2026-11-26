"""Nominal hardware envelope screening. No commercial part qualification."""
import math
from modular_arm.v4 import joint,flush_arm
from modular_arm.v3.locking_core import Core,cyl

def spring_candidate(od,free_length,solid_height,rate):
    values=(od,free_length,solid_height,rate)
    if not all(math.isfinite(v) and v>0 for v in values):
        raise ValueError("positive finite spring dimensions required")
    # Provisional design margins, not manufacturing tolerances.
    issues=[]
    if od>18: issues.append("outside_nominal_envelope")
    if free_length<=13: issues.append("no_preload")
    if solid_height>5: issues.append("insufficient_solid_height_margin")
    if solid_height>=free_length: issues.append("invalid_spring_lengths")
    return {"issues":issues,"preload_N":rate*(free_length-13),
            "full_press_N":rate*(free_length-6),
            "solid_height_margin_mm":6-solid_height}

def report():
    c=Core()
    a,b=flush_arm.finished_flush(1),flush_arm.finished_flush(5)
    poses=[]
    for travel in (0.,3.5,7.):
        spring=joint.spring_space(travel)
        button=flush_arm.button(travel)
        poses.append({"travel_mm":travel,"spring_length_mm":13-travel,
            "spring_intersection_a_mm3":spring.intersect(a).Volume(),
            "spring_intersection_b_mm3":spring.intersect(b).Volume(),
            "spring_intersection_button_mm3":spring.intersect(button).Volume(),
            "button_intersection_a_mm3":button.intersect(a).Volume(),
            "button_intersection_b_mm3":button.intersect(b).Volume()})
    pins=[]
    for name,pin in joint.screws().items():
        pins.append({"name":name,"intersection_a_mm3":pin.intersect(a).Volume(),
            "intersection_b_mm3":pin.intersect(b).Volume(),
            "axial_capture_test_mm3":pin.translate((0,0,2)).intersect(a).Volume()})
    return {"spring":{"od_max_mm":18,"installed_mm":13,"compressed_mm":6,
              "illustrative_candidate":spring_candidate(c.spring_od,c.spring_free,c.spring_solid,c.spring_rate)},
            "retainers":{"count":3,"nominal_bore_mm":2.5,"smooth_gauge_mm":2.4,
              "gauge_length_mm":5.8,"groove_radial_depth_mm":1.5,
              "tip_radial_clearance_mm":.5,"nominal_engagement_mm":1.,
              "status":"thread and commercial screw not qualified","checks":pins},
            "poses":poses,"status":"nominal CAD screening only"}
