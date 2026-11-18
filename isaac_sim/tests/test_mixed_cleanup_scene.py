from collections import Counter
from pathlib import Path
import sys

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from mixed_cleanup_scene import mixed_prop_specs
from floor_cleanup import floor_groups
from mixed_cleanup_scene import mixed_pickup_yaws
from mixed_cleanup_scene import mixed_rack_footprint
from mixed_cleanup_scene import mixed_rack_yaws
from floor_cleanup import CleanupQueue
import math


def test_long_mixed_props_use_crosswise_carry_approaches():
    assert all(abs(math.cos(yaw))<1e-10 for yaw in mixed_pickup_yaws('beam'))
    assert mixed_pickup_yaws('tool')==mixed_pickup_yaws('beam')
    assert len(mixed_pickup_yaws('carton'))==4


def test_central_rack_uses_the_face_nearest_the_source_prop():
    assert mixed_rack_yaws((0.,math.pi),3.,-1.,1.)[0]==math.pi
    assert mixed_rack_yaws((0.,math.pi),-3.,-1.,1.)[0]==0.
    assert mixed_rack_yaws((math.pi,),3.,-9.,-8.)==(math.pi,)


def test_mixed_reservations_keep_room_for_small_settling_changes():
    queue=CleanupQueue([{'id':n,'kind':'debris','paths':[n]} for n in ('a','b')])
    queue.claim('a',1,{'path':'rack','box':mixed_rack_footprint((0,0,.2,.2))})
    queue.claim('b',2,{'path':'rack','box':mixed_rack_footprint((.36,0,.56,.2))})
    queue.complete('a',True,(.003,-.003,.203,.197))
    assert queue.items['a']['state']=='stored'


def test_mixed_layout_has_four_tasks_in_each_requested_family():
    specs=mixed_prop_specs(20)
    assert Counter(s['label'] for s in specs)=={
        'Rear-left carton':4,'Aisle rubble':4,'Screwdriver':4,'Rear beam':4,'Rear-right carton':4}
    assert len({s['name'] for s in specs})==20
    assert len({s['position'] for s in specs})==20
    assert all(max(s['size'])<=.62 for s in specs)
    tools=[s for s in specs if s['label']=='Screwdriver']
    assert all(s['position'][0]+s['size'][0]/2+.24 < -1.9 for s in tools)


def test_screwdriver_handle_and_shaft_are_one_task():
    class Prim:
        def __init__(self,name):self.name=name
        def GetName(self):return self.name
        def GetPath(self):return '/Props/'+self.name
    names=[]
    for spec in mixed_prop_specs(20):
        names.append(spec['name'])
        if spec['label']=='Screwdriver':names.append(spec['name'].replace('_Handle_','_Shaft_'))
    groups,unknown=floor_groups([Prim(n) for n in names])
    assert len(groups)==20 and not unknown
    assert Counter(g['kind'] for g in groups)=={'carton':8,'debris':4,'tool':4,'beam':4}
    assert sum(len(g['paths']) for g in groups)==24
