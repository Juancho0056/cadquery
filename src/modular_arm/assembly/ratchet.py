"""Integrated contact prototype. All motion is manually prescribed.

No physical selector or return spring; no load rating.
"""
import math
import cadquery as cq
from modular_arm.components.joint import make_fork
from modular_arm.assembly.joint_keeper import joint_with_keeper
from modular_arm.mechanisms.pivot_contact import PivotContactConfig,make_contact_rotor,make_contact_pawl


def cylinder(x,y,z,d,h):
    return cq.Solid.makeCylinder(d/2,h,cq.Vector(x,y,z))


def make_contact_fork(c=None):
    c=c or PivotContactConfig()
    j=c.joint
    p=make_fork(j).val()
    for x,y,d in [*c.support_centers,(-c.lock_radius_mm,0,c.lock_bore_mm)]:
        p=p.cut(cylinder(x,y,-1,d,j.stack_height_mm+2))
    return p


def ratchet_parts(c=None, *, angle_deg=0., lower_release=0., upper_release=0., lock_inserted=True):
    c=c or PivotContactConfig()
    j=c.joint
    a=j.cartridge
    z=a.head_thickness_mm+a.fork_wall_mm+a.axial_clearance_per_side_mm
    if not math.isfinite(angle_deg):
        raise ValueError("nonfinite rotor angle")
    parts=joint_with_keeper(j)
    del parts["ring"]
    parts["fork"]=make_contact_fork(c).translate(cq.Vector(0,0,a.head_thickness_mm))
    parts["rotor"]=make_contact_rotor(c).rotate((0,0,0),(0,0,1),angle_deg).translate(cq.Vector(0,0,z))
    for name,reverse,q in [("lower_pawl",False,lower_release),("upper_pawl",True,upper_release)]:
        parts[name]=make_contact_pawl(c,release_deg=q,reverse=reverse).translate(cq.Vector(0,0,z))
    for name,x,y,d in [("lower_pivot",c.pivot_x_mm,c.pivot_y_mm,c.pivot_diameter_mm),
                       ("upper_pivot",-c.pivot_x_mm,c.pivot_y_mm,c.pivot_diameter_mm),
                       ("lower_stop",c.stop_x_mm,c.stop_y_mm,2*c.stop_radius_mm),
                       ("upper_stop",-c.stop_x_mm,c.stop_y_mm,2*c.stop_radius_mm)]:
        parts[name]=cylinder(x,y,a.head_thickness_mm,d,j.stack_height_mm)
    if lock_inserted:
        x=-c.lock_radius_mm
        stem=cylinder(x,0,a.head_thickness_mm,c.lock_diameter_mm,j.stack_height_mm)
        head=cylinder(x,0,a.head_thickness_mm+j.stack_height_mm,c.lock_head_diameter_mm,c.lock_head_thickness_mm)
        parts["angular_lock"]=stem.fuse(head)
    return parts
