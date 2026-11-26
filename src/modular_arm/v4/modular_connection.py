"""Experimental removable axial connection, separate from integral V4.
Head socket and keyed arm tenon; all dimensions in mm. Not load rated.
"""
import math,json
from pathlib import Path
import cadquery as cq
from modular_arm.v4 import flush_arm,joint,integrated_arm as ia

JOIN=46.
LENGTH=16.
RADIUS=10.
FLAT=7.
CLEARANCE=.25
END_GAP=.3
PIN_AT=38.
PIN_RADIUS=2.15
SCREW_RADIUS=2.
SCREW_LENGTH=35.
SEAT_Y=12.8
SEAT_RADIUS=4.7
WASHER_RADIUS=4.5
WASHER_THICKNESS=.8
HEAD_RADIUS=3.8
HEAD_HEIGHT=2.2
NUT_RADIUS=4.1
NUT_HEIGHT=5.

V=cq.Vector

def direction(index):
    if index not in (1,5):
        raise ValueError("unsupported arm")
    return -1 if index==1 else 1

def axial_box(index,lo,hi,y=60.,zlo=-60.,zhi=80.):
    d=direction(index)
    return cq.Solid.makeBox(hi-lo,2*y,zhi-zlo,V(min(d*lo,d*hi),-y,zlo))

def keyed(index,lo,hi,clearance=0.):
    d=direction(index)
    z=flush_arm.AXES[index]
    s=cq.Solid.makeCylinder(RADIUS+clearance,hi-lo,V(d*lo,0,z),V(d,0,0))
    return s.intersect(axial_box(index,lo,hi,zhi=z+FLAT+clearance)).clean()

def pin_bore(index):
    return cq.Solid.makeCylinder(PIN_RADIUS,40,V(direction(index)*PIN_AT,-20,flush_arm.AXES[index]),V(0,1,0))

def cross_cylinder(index,radius,y,length):
    return cq.Solid.makeCylinder(radius,length,V(direction(index)*PIN_AT,y,flush_arm.AXES[index]),V(0,1,0))

def bearing_seats(index):
    return (cross_cylinder(index,SEAT_RADIUS,-25.,25.-SEAT_Y),
            cross_cylinder(index,SEAT_RADIUS,SEAT_Y,25.-SEAT_Y))

def head(index):
    s=flush_arm.finished_flush(index)
    s=s.intersect(axial_box(index,-60,JOIN))
    # Blind socket opens at the distal face. No cuts into the joint core.
    s=s.cut(keyed(index,JOIN-LENGTH-END_GAP,JOIN+1,CLEARANCE))
    for seat in bearing_seats(index):
        s=s.cut(seat)
    return valid(s.cut(pin_bore(index)).clean(),"head")

def tenon(index):
    # Overlap 1.5 mm into the retained original section, as in integral V4.
    return valid(keyed(index,JOIN-LENGTH,JOIN+1.5).cut(pin_bore(index)).clean(),"tenon")

def valid(s,label):
    if not s.isValid() or len(s.Solids())!=1:
        raise ValueError(f"invalid modular {label}")
    return s

def coupon_arm(index):
    d=direction(index)
    stub=cq.Solid.makeCylinder(flush_arm.RADIUS,18,V(d*JOIN,0,flush_arm.AXES[index]),V(d,0,0))
    return valid(stub.fuse(tenon(index)).clean(),"coupon arm")

def arm(archive,index):
    tm=ia.mesh_library()
    body=ia.align_body(ia.crop_terminal(ia.original_mesh(archive,index),index),index,
                       axis_z=flush_arm.AXES[index],clocking=180.)
    male=ia.cad_mesh(tenon(index))
    overlap=tm.boolean.intersection([body,male],engine="manifold")
    if ia.intersection_volume(overlap)<=1:
        raise ValueError("insufficient original body overlap")
    result=ia.require_solid(tm.boolean.union([body,male],engine="manifold"),"modular arm")
    preservation=ia.preservation_report(body,result,index)
    if max(preservation.values())>.05:
        raise ValueError(f"original longitudinal body changed: {preservation}")
    return result

def pin_gauge(index):
    """Smooth major-diameter envelope, not a modeled thread."""
    return cross_cylinder(index,SCREW_RADIUS,-SEAT_Y-WASHER_THICKNESS,SCREW_LENGTH)

def annular_envelope(index,radius,y,height):
    return cross_cylinder(index,radius,y,height).cut(
        cross_cylinder(index,PIN_RADIUS,y-.1,height+.2)).clean()

def connection_hardware(index):
    y=-SEAT_Y-WASHER_THICKNESS
    screw=pin_gauge(index).fuse(cross_cylinder(index,HEAD_RADIUS,y-HEAD_HEIGHT,HEAD_HEIGHT)).clean()
    return {"screw":valid(screw,"screw envelope"),
            "washer_head":annular_envelope(index,WASHER_RADIUS,y,WASHER_THICKNESS),
            "washer_nut":annular_envelope(index,WASHER_RADIUS,SEAT_Y,WASHER_THICKNESS),
            "nut":annular_envelope(index,NUT_RADIUS,SEAT_Y+WASHER_THICKNESS,NUT_HEIGHT)}

def hardware_dimensions():
    grip=2*(SEAT_Y+WASHER_THICKNESS)
    return {"thread":"M4 x 0.7","length_under_head_mm":SCREW_LENGTH,
            "head_envelope_mm":[2*HEAD_RADIUS,HEAD_HEIGHT],
            "nut_envelope_mm":[2*NUT_RADIUS,NUT_HEIGHT],
            "washer_envelope_mm":[2*WASHER_RADIUS,WASHER_THICKNESS],
            "bearing_seat_diameter_mm":2*SEAT_RADIUS,
            "grip_mm":grip,"thread_projection_beyond_nut_mm":SCREW_LENGTH-grip-NUT_HEIGHT,
            "nominal_socket_side_wall_at_seat_center_mm":SEAT_Y-RADIUS-CLEARANCE,
            "status":"nominal envelopes; verify supplier dimensions and actual bearing contact"}

def parts(archive=None,angle=0.,travel=0.):
    joint.parts(angle,travel)
    result={}
    for i,label in ((1,"a"),(5,"b")):
        h=ia.cad_mesh(head(i))
        a=ia.cad_mesh(coupon_arm(i)) if archive is None else arm(archive,i)
        hardware={f'connection_{label}_{n}':ia.cad_mesh(s) for n,s in connection_hardware(i).items()}
        if i==5:
            transform=ia.mesh_library().transformations.rotation_matrix(math.radians(angle),(0,0,1))
            for m in (h,a,*hardware.values()):
                m.apply_transform(transform)
        result.update({f"head_{label}":h,f"tube_{label}":a,**hardware})
    result["button"]=ia.cad_mesh(flush_arm.button(travel))
    result.update({n:ia.cad_mesh(s) for n,s in joint.screws(angle).items()})
    return result

def export(destination,archive=None,angle=0.,travel=0.):
    p=parts(archive,angle,travel)
    defects=ia.audit(p)
    if defects:
        raise ValueError(f"modular interference mm3: {defects}")
    neutral=p if (angle,travel)==(0.,0.) else parts(archive)
    dest=Path(destination)
    preview=dest/"assembly_preview"
    printable=dest/"prototype_printable"
    for folder in (preview,printable):
        folder.mkdir(parents=True,exist_ok=True)
    names=("head_a","head_b","tube_a","tube_b","button")
    for label in ("a","b"):
        (preview/f"connection_pin_{label}.stl").unlink(missing_ok=True)
    for n,m in p.items():
        m.export(preview/(n+".stl"))
    for n in names:
        m=neutral[n].copy()
        if n!="button":
            # Head socket up, tube tenon up; evaluate actual supports in slicer.
            sign=(1 if n.endswith("_a") else -1) if n.startswith("head") else (-1 if n.endswith("_a") else 1)
            m.apply_transform(ia.mesh_library().transformations.rotation_matrix(sign*math.pi/2,(0,1,0)))
        m.apply_translation(-m.bounds[0])
        m.export(printable/(n+".stl"))
    manifest={"status":"unloaded_fit_prototype_only","variant":"coupon" if archive is None else "reference_tubes",
              "printable":names,"connection":{"engagement_mm":LENGTH,"diameter_mm":2*RADIUS,
              "flat_offset_mm":FLAT,"radial_clearance_mm":CLEARANCE,"end_clearance_mm":END_GAP,
              "cross_bore_mm":2*PIN_RADIUS,"fastener":hardware_dimensions()},
              "hardware":"Two M4 x 35 button-head screws, two M4 prevailing-torque nuts, four M4 washers, three joint retainers and one compression spring. Metal envelopes are preview only; verify real fastener dimensions, seating and preload.",
              "pose":{"angle":angle,"travel":travel},"notes":"No load rating. Full telescoping assembly not validated. No changes to integral V4."}
    (dest/"manifest.json").write_text(json.dumps(manifest,indent=2),encoding="utf-8")
    return manifest
