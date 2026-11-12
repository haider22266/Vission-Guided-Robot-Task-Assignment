import math
import sys
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from convoy import Convoy, point_on_path, path_slice


class Op:
    def Set(self,value):
        self.value=value


def test_shared_route_keeps_spacing_and_altitudes():
    c=Convoy.__new__(Convoy)
    c.path=[(0.,0.),(5.,0.)]
    c.spacing=.7
    c.speed=.2
    c.direction=1
    c.obstacles=[]
    robots=[{} for _ in range(4)]
    explorers=[SimpleNamespace(position=(0,0,z),radius=.1,vertical_offsets=(-.05,.05)) for z in (1.2,1.4,.1,.1)]
    c.response=SimpleNamespace(explorers=explorers,catalog={'robots':robots},ops=[(Op(),Op()) for _ in robots],
                               Gf=SimpleNamespace(Vec3d=lambda *v:v))
    assert c.update_positions(0.,initial=True)
    before=[e.position for e in explorers]
    assert c.update_positions(.5)
    for old,e in zip(before,explorers):
        assert math.isclose(e.position[0]-old[0],.5)
        assert e.position[1]==0 and e.position[2]==old[2]
    assert all(math.isclose(b.position[0]-a.position[0],.7) for a,b in zip(explorers,explorers[1:]))


def test_arc_length_tracks_bends():
    point,yaw=point_on_path([(0,0),(2,0),(2,2)],3)
    assert point==(2.,1.) and yaw==90


def test_prefix_slice_preserves_bends_and_length():
    assert path_slice([(0,0),(2,0),(2,3)],1,4)==[(1.,0.),(2,0),(2.,2.)]


def test_coverage_extension_keeps_formation_and_reaches_new_area():
    c=Convoy.__new__(Convoy)
    c.path=[(0.,0.),(5.,0.)]
    c.spacing=1.
    c.span=3.
    c.progress=2.
    c.direction=1
    c.order=list(range(4))
    c.speed=.3
    c.length=5.
    c.limit=2.
    c.radius=.1
    c.obstacles=[]
    c.visits={}
    c.legs=1
    c.coverage_points=[(8.,4.),(5.,4.),(3.,3.)]
    explorers=[SimpleNamespace(position=(2+i,0.,.2*i),radius=.1,vertical_offsets=(-.02,.02)) for i in range(4)]
    c.response=SimpleNamespace(explorers=explorers,bounds=(-10,-10,10,10),
        catalog={'robots':[{} for _ in range(4)],'navigation':{}},
        ops=[(Op(),Op()) for _ in range(4)],Gf=SimpleNamespace(Vec3d=lambda *v:v))
    before=[e.position for e in explorers]
    assert c.extend_route()
    assert c.legs==2 and c.limit>.25
    assert c.update_positions(0.)
    assert [e.position for e in explorers]==before
    assert any(p[1]>0 for p in c.path)
    for _ in range(100):
        c.step(.1)
    assert max(e.position[1] for e in explorers)>0
