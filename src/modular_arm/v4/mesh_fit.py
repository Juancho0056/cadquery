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

def fit_report(archive,angle=0.,travel=0.):
    p=parts(angle,travel)
    original={f"obj_{i}":polydata(aligned_triangles(archive,i,angle)) for i in (1,5)}
    generated={n:cad_mesh(s) for n,s in p.items()}
    return [{"original":a,"cad":b,"surface_contacts":contacts(m,n)}
            for a,m in original.items() for b,n in generated.items()]
