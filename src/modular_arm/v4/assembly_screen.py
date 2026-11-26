"""Prescribed assembly path checks for flush V4, in mm.
Rigid nominal CAD only: no thread strength, friction or physical certification.
"""
import math
import cadquery as cq
from modular_arm.v4 import joint,flush_arm
from modular_arm.v3.locking_core import Core,cyl

EPS=1e-6

def overlap(a,b):
    return float(a.intersect(b).Volume())

def report():
    a=flush_arm.finished_flush(1)
    b=flush_arm.finished_flush(5)
    p=flush_arm.button()
    c=Core()
    # Spring inserted from above through stationary guide, before button.
    # OD16 is candidate only; actual coil must meet compressed-height spec.
    spring=cyl(8,-7,13)
    spring_path=[]
    for lift in range(0,41,2):
        spring_path.append({"lift_mm":lift,"collision_mm3":overlap(a,spring.translate((0,0,lift)))})
    button_path=[]
    for lift in range(0,41,2):
        button_path.append({"lift_mm":lift,"collision_mm3":overlap(a,p.translate((0,0,lift)))})
    rotor_path=[]
    for lift in range(0,41,2):
        q=b.translate((0,0,lift))
        rotor_path.append({"lift_mm":lift,"fixed_collision_mm3":overlap(a,q),
                           "button_collision_mm3":overlap(p,q)})
    retainers=[]
    for angle,(name,pin) in zip(joint.radial_angles(),joint.screws().items()):
        t=math.radians(angle)
        d=cq.Vector(math.cos(t),math.sin(t),0)
        path=[]
        for lift in range(0,13):
            q=pin.translate(d*lift)
            path.append({"withdrawal_mm":lift,"fixed_collision_mm3":overlap(a,q),
                         "rotor_collision_mm3":overlap(b,q)})
        # Tool access envelope, outside the housing only. Not a tool model.
        tool=joint.radial_cylinder(2.,28.1,15.,angle)
        retainers.append({"name":name,"insertion_path":path,
                          "external_tool_access_collision_mm3":overlap(b,tool)})
    # A lift of the button must be stopped by the upper housing shoulder.
    capture=[{"lift_mm":lift,"rotor_intersection_mm3":overlap(b,p.translate((0,0,lift)))}
             for lift in (.1,.5,1.,2.)]
    issues=[]
    for label,rows in (("spring",spring_path),("button",button_path),("rotor",rotor_path)):
        for row in rows:
            if any(v>EPS for k,v in row.items() if "collision" in k):
                issues.append({"stage":label,**row})
    for r in retainers:
        if r["external_tool_access_collision_mm3"]>EPS:
            issues.append({"stage":"tool_access","retainer":r["name"]})
        for row in r["insertion_path"]:
            if any(v>EPS for k,v in row.items() if "collision" in k):
                issues.append({"stage":"retainer_insertion","retainer":r["name"],**row})
    if not any(r["rotor_intersection_mm3"]>EPS for r in capture):
        issues.append({"stage":"button_capture","reason":"no positive stop detected"})
    return {"status":"sampled_nominal_assembly_path_only","spring_path":spring_path,
            "button_path":button_path,"rotor_path":rotor_path,"retainers":retainers,
            "button_capture":capture,"issues":issues}
