"""ASM-001: removable lid with two accessible cantilever hooks. mm."""
from pathlib import Path
import cadquery as cq


def box(x, y, z, cx, cy, cz):
    return cq.Workplane('XY').box(x,y,z).translate((cx,cy,cz))


def build(clearance=0.30, finger_thickness=1.6, engagement=1.0):
    if not 0.15 <= clearance <= 0.6 or not 1.2 <= finger_thickness <= 2.0 or not 0.6 <= engagement <= 1.4:
        raise ValueError('Outside coupon parameter range')
    # Outer 40 x 30 x 20; wall 3; floor 2.
    body = box(40,30,20,0,0,10).cut(box(34,24,20,0,0,12))
    lid = box(40,30,2.4,0,0,21.2)
    outer = 17-clearance
    inner = outer-finger_thickness
    tip = 17+engagement
    finger = box(finger_thickness,8,14, (inner+outer)/2,0,13)
    # Lower insertion ramp; upper horizontal shoulder resists withdrawal.
    hook = cq.Workplane('XZ').polyline([(inner,6),(outer,6),(tip,8),(tip,9),(inner,9)]).close().extrude(4,both=True)
    finger = finger.union(hook)
    for angle in (0,180):
        lid = lid.union(finger.rotate((0,0,0),(0,0,1),angle))
        window = box(5,8+2*clearance,3+2*clearance,19,0,7.5)
        body = body.cut(window.rotate((0,0,0),(0,0,1),angle))
    return {'body':body,'lid':lid}


def export_all():
    for c in (0.2,0.3,0.4):
        dest=Path(__file__).parent/'exports'/f'clearance_{c:.2f}'
        dest.mkdir(parents=True,exist_ok=True)
        for name,part in build(c).items():
            assert part.val().isValid()
            # Flip lid onto its exterior surface for printing; STEP retains assembly coordinates.
            printable=part if name=='body' else part.rotate((0,0,0),(1,0,0),180).translate((0,0,22.4))
            cq.exporters.export(part,str(dest/f'{name}.step'))
            cq.exporters.export(printable,str(dest/f'{name}.stl'))

if __name__=='__main__':
    export_all()
