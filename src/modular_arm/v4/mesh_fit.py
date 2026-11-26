"""Surface intersection and containment screening against original STL.
Uses VTK shipped with CadQuery. Not a continuous-motion or load certificate.
"""
import numpy as np
import vtk
from vtk.util.numpy_support import numpy_to_vtk,numpy_to_vtkIdTypeArray
from modular_arm.v4.reference_preview import aligned_triangles
from modular_arm.v4.rigid_ends import parts

def polydata(triangles):
    t=np.asarray(triangles,dtype=float)
    p=vtk.vtkPoints()
    p.SetData(numpy_to_vtk(t.reshape(-1,3),deep=True))
    cells=vtk.vtkCellArray()
    cells.SetCells(len(t),numpy_to_vtkIdTypeArray(np.column_stack((np.full(len(t),3),np.arange(len(t)*3).reshape(-1,3))).astype(np.int64).ravel(),deep=True))
    mesh=vtk.vtkPolyData()
    mesh.SetPoints(p)
    mesh.SetPolys(cells)
    clean=vtk.vtkCleanPolyData()
    clean.SetInputData(mesh)
    clean.Update()
    return clean.GetOutput()

def cad_mesh(shape):
    vertices,faces=shape.tessellate(.08,.08)
    return polydata(np.array([v.toTuple() for v in vertices])[np.asarray(faces)])

def contacts(a,b):
    f=vtk.vtkCollisionDetectionFilter()
    f.SetInputData(0,a)
    f.SetInputData(1,b)
    f.SetMatrix(0,vtk.vtkMatrix4x4())
    f.SetMatrix(1,vtk.vtkMatrix4x4())
    f.SetCollisionModeToAllContacts()
    f.SetBoxTolerance(0.)
    f.SetCellTolerance(0.)
    f.Update()
    return f.GetNumberOfContacts()

def enclosed_vertices(a,b,tolerance=0.0001):
    """Count sampled mesh vertices strictly inside the other closed surface.
    Near-surface points are excluded to avoid classifying contact as containment.
    """
    selector=vtk.vtkSelectEnclosedPoints()
    selector.SetInputData(a)
    selector.SetSurfaceData(b)
    selector.SetTolerance(tolerance)
    selector.CheckSurfaceOn()
    selector.Update()
    flags=selector.GetOutput().GetPointData().GetArray("SelectedPoints")
    if flags is None:
        raise ValueError("containment check failed; check mesh closure")
    from vtk.util.numpy_support import vtk_to_numpy
    ids=np.flatnonzero(vtk_to_numpy(flags))
    distance=vtk.vtkImplicitPolyDataDistance()
    distance.SetInput(b)
    return sum(abs(distance.EvaluateFunction(a.GetPoint(int(i))))>.02 for i in ids)

def nearest_sample(a,b):
    """Minimum vertex-to-triangle distance, upper bound on surface gap."""
    from vtk.util.numpy_support import vtk_to_numpy
    pts=vtk_to_numpy(a.GetPoints().GetData())
    # Deterministic bounded sampling, include extrema; not exact minimum.
    ids=np.unique(np.r_[np.linspace(0,len(pts)-1,min(4096,len(pts)),dtype=int),pts.argmin(0),pts.argmax(0)])
    distance=vtk.vtkImplicitPolyDataDistance()
    distance.SetInput(b)
    return min(abs(distance.EvaluateFunction(pts[i])) for i in ids)

def pair_report(a,b):
    count=contacts(a,b)
    return {"surface_contacts":count,
            "a_vertices_inside_b":enclosed_vertices(a,b),
            "b_vertices_inside_a":enclosed_vertices(b,a),
            "sampled_gap_mm":min(nearest_sample(a,b),nearest_sample(b,a))}

def fit_report(archive,angle=0.,travel=0.):
    p=parts(angle,travel)
    original={f"obj_{i}":polydata(aligned_triangles(archive,i,angle)) for i in (1,5)}
    generated={n:cad_mesh(s) for n,s in p.items()}
    result=[]
    for a,m in original.items():
        for b,n in generated.items():
            row={"original":a,"cad":b,**pair_report(m,n)}
            row["expected_mate"]=(a,b) in (("obj_1","arm_a"),("obj_5","arm_b"))
            result.append(row)
    result.append({"original":"obj_1","cad":"obj_5",**pair_report(original["obj_1"],original["obj_5"]),"expected_mate":False})
    return result

def failures(report):
    result=[]
    for row in report:
        if any(row[k] for k in ("surface_contacts","a_vertices_inside_b","b_vertices_inside_a")):
            result.append({"pair":[row["original"],row["cad"]],"reason":"intersection_or_containment"})
        if row["expected_mate"] and row["sampled_gap_mm"]>1.:
            result.append({"pair":[row["original"],row["cad"]],"reason":"mating_gap_above_screening_threshold"})
    return result
