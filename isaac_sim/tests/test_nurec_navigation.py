import sys
from pathlib import Path
from types import SimpleNamespace

sys.path[:0]=[str(Path(__file__).resolve().parents[1]),
              str(Path(__file__).resolve().parents[2]/'src'/'disaster_mrta')]
from nurec_navigation import raster_rectangles, NuRecResponse, free_points
from navigation import Explorer, clear_segment
from disaster_mrta.tasks import Task


def test_floor_exclusion_and_height_filter():
    triangles=[((0,0,0),(4,4,.02)), ((1,1,.2),(2,2,1)), ((3,3,2),(4,4,3))]
    obstacles=raster_rectangles(triangles,(0,0,4,4),.1,1.2)
    assert clear_segment((.5,.5),(.5,.5),obstacles,0)
    assert not clear_segment((1.5,1.5),(1.5,1.5),obstacles,0)
    assert clear_segment((3.5,3.5),(3.5,3.5),obstacles,0)
    assert all(clear_segment(p,p,obstacles,.2) for p in free_points((0,0,4,4),obstacles,.2))


class Op:
    def Set(self,value):
        self.value=value


def response(obstacles):
    robot={'robot_id':'delivery','robot_type':'UGV','capabilities':['supply_delivery'],
           'x':0.,'y':0.,'z':0.,'battery':100.,'available':True,'healthy':True,
           'communication_available':True,'current_task':None,'speed_mps':1.,
           'energy_per_meter':.1,'execution_energy_per_second':.1}
    r=NuRecResponse.__new__(NuRecResponse)
    r.catalog={'robots':[robot]}
    r.tasks=[Task('request','supply_delivery',(1.,0.),.8,'supply_delivery',.2)]
    r.explorers=[Explorer([(0,0,0),(0,0,0)],1,.1,(0,.3))]
    r.maps=[obstacles]
    r.bounds=(-12,-15,12,15)
    r.ops=[(Op(),Op())]
    r.Gf=SimpleNamespace(Vec3d=lambda *v:v)
    r.candidates=[[(0,0),(1,0)]]
    r.remaining,r.energy={},{}
    r.reserve=10.
    r.elapsed,r.explore_first=0.,0.
    r.visits=[{}]
    return r


def test_delivery_runs_only_with_reachable_route_and_consumes_energy():
    r=response([])
    for _ in range(20):
        r.step(.1)
    assert r.tasks[0].status=='completed'
    assert r.tasks[0].assigned_robot=='delivery'
    assert r.catalog['robots'][0]['battery']<100


def test_blocked_delivery_stays_pending():
    r=response([(.5,-1,1.5,1)])
    r.step(.1)
    assert r.tasks[0].status=='pending'
    assert r.catalog['robots'][0]['current_task'] is None


def test_beginning_exploration_moves_before_assigning_tasks():
    r=response([])
    r.explore_first=2.
    r.step(.1)
    assert r.explorers[0].position[0]>0
    assert r.tasks[0].status=='pending'
    for _ in range(21):
        r.step(.1)
    assert r.tasks[0].status!='pending'


def test_camera_area_points_are_distributed():
    from camera_tour import distributed_points
    points=[(x,y) for x in range(5) for y in range(5)]
    shots=distributed_points(points)
    assert len(set(shots))==4
    assert (0,0) in shots and (4,4) in shots
