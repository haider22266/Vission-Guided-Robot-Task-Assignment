import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from navigation import Explorer, clear_segment, plan, obstacles_for


def test_planner_detours_with_clearance():
    obstacles=[(-.5,-1,.5,1)]
    start=(-3.,0.)
    path=plan(start,(3.,0.),obstacles,.35,bounds=(-5,-5,5,5))
    assert path and len(path)>1
    for point in path:
        assert clear_segment(start,point,obstacles,.35)
        start=point
    assert start == (3.,0.)


def test_blocked_goal_has_no_route():
    assert plan((-3,0),(0,0),[(-1,-1,1,1)],.2)==[]


def test_no_corner_cutting_or_tunneling():
    assert not clear_segment((-2,0),(2,0),[(-.01,-1,.01,1)],.1)
    assert not clear_segment((0,0),(2,2),[(.8,.8,1.2,1.2)],.2)


def test_new_obstacle_stops_robot_and_removal_allows_motion():
    e=Explorer([(0,0,0),(3,0,0)],1,.2,(0,.5))
    e.step(.1,[])
    before=e.position
    # An object overlapping the current clearance must cause a stop.
    e.step(.1,[(-1,-1,1,1)])
    assert e.position == before and e.state == 'blocked'
    for _ in range(20):
        e.step(.1,[])
    assert e.position[0] > before[0]


def test_height_filter_and_peer_clearance():
    e=Explorer([(0,0,2),(1,0,2)],1,.2,(-.1,.1))
    other=Explorer([(0,0,0),(1,0,0)],1,.3,(0,.5))
    assert obstacles_for(e,[((-1,-1,0),(1,1,1))],[e,other]) == []
    other.position=(0,0,2)
    assert len(obstacles_for(e,[],[e,other])) == 1
