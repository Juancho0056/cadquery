"""Probeta de retención axial mediante dos nervios anulares y carcasa partida.
No es un snap-fit: las dos mitades se mantienen juntas externamente durante el ensayo.
Todas las dimensiones están en mm.
"""
from pathlib import Path
import cadquery as cq


def build(clearance=0.30, shaft_radius=6.0, rib_height=1.2, rib_width=2.4):
    if not 0 < clearance < rib_height or shaft_radius <= 0 or not 0 < rib_width < 5:
        raise ValueError('Parámetros fuera del dominio de la probeta')
    length = 24.0
    outer = shaft_radius + rib_height + clearance + 3.0
    shaft = cq.Workplane('XY').circle(shaft_radius).extrude(length)
    shell = cq.Workplane('XY').circle(outer).circle(shaft_radius+clearance).extrude(length)
    for z in (6., 16.):
        shaft = shaft.union(cq.Workplane('XY').workplane(offset=z).circle(shaft_radius+rib_height).extrude(rib_width))
        groove = cq.Workplane('XY').workplane(offset=z-clearance).circle(shaft_radius+rib_height+clearance).extrude(rib_width+2*clearance)
        shell = shell.cut(groove)
    half_a = shell.intersect(cq.Workplane('XY').box(2*outer+2,outer+1,length+2).translate((0,(outer+1)/2,length/2)))
    half_b = shell.intersect(cq.Workplane('XY').box(2*outer+2,outer+1,length+2).translate((0,-(outer+1)/2,length/2)))
    return {'shaft':shaft,'shell_a':half_a,'shell_b':half_b}

if __name__ == '__main__':
    root=Path(__file__).parent/'exports'
    for c in (0.2,0.3,0.4):
        dest=root/f'clearance_{c:.2f}'
        dest.mkdir(parents=True,exist_ok=True)
        for name, part in build(c).items():
            assert part.val().isValid()
            for ext in ('step','stl'):
                cq.exporters.export(part,str(dest/f'{name}.{ext}'))
