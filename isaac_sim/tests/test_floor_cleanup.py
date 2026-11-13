import sys
from pathlib import Path
import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from floor_cleanup import CleanupQueue,floor_groups,FairTripGate,RouteReservations,beam_fits_support,route_prefix,upper_face,supported_centres,idle_clear_due


def test_idle_aisle_clearing_preserves_loads_and_bounds_retries():
    assert idle_clear_due((2,0),(0,-6),False,-100,0)
    assert not idle_clear_due((2,0),(0,-6),True,-100,0)
    assert not idle_clear_due((0,-6),(0,-6),False,-100,0)
    assert not idle_clear_due((2,0),(0,-6),False,10,20)
    assert idle_clear_due((2,0),(0,-6),False,10,40)
    routes=RouteReservations()
    assert not routes.reserve(4,[(0,0),(3,0)],.6,{1:((2,0),.6)})
    assert routes.reserve(4,[(0,0),(3,0)],.6,{1:((0,-6),.6)})


def test_alternative_placement_centres_cover_support_without_overhang():
    positions=supported_centres((0.,0.),(1.,2.),(.2,.3),(.3,.4))
    assert positions[0]==(.3,.4)
    assert any(y>1.5 for x,y in positions)
    assert any(x>.7 for x,y in positions)
    assert all(.215<=x<=.785 and .315<=y<=1.685 for x,y in positions)
    assert supported_centres((0.,0.),(.3,.3),(.2,.2),(0.,0.))==[]


@pytest.mark.parametrize('directions,expected',[
    ([(1,0,0),(0,1,0),(0,0,1)],(2,1.)),
    ([(0,0,-2),(0,3,0),(4,0,0)],(0,-1.)),
    ([(10,0,1),(0,0,.2),(0,1,0)],(1,1.)),
])
def test_grasp_selects_actual_upper_face_after_tipping(directions,expected):
    axis,sign,tangent_axes=upper_face(directions)
    assert (axis,sign)==expected
    assert sorted([axis]+tangent_axes)==[0,1,2]


def groups():
    return [{'id':x,'kind':'carton','paths':[x]} for x in ('a','b')]


def test_rejecting_preferred_object_keeps_shared_pool_available():
    queue=CleanupQueue(groups())
    queue.reject(1,['a'],'No feasible preferred pickup')
    assert queue.available(1)==['b']
    assert queue.summary()['counts']['pending']==2


def test_shorter_route_can_progress_without_reserving_blocked_future_space():
    routes=RouteReservations()
    path=[(0.,0.),(2.,0.)]
    peers={2:((2.,0.),.6)}
    assert not routes.reserve(1,path,.6,peers)
    prefix=route_prefix(path,.5)
    assert routes.reserve(1,prefix,.6,peers)
    assert prefix[-1]==(.5,0.)  # Execute the granted endpoint, not the old goal.


def test_traffic_before_pickup_releases_object_and_slot_for_other_robot():
    queue=CleanupQueue(groups())
    queue.claim('a',1,{'box':(0,0,1,1),'path':'rack'})
    queue.fail('a','Traffic deferred',before_manipulation=True)
    assert queue.summary()['counts']['failed']==0
    assert 'a' in queue.available(2) and 'b' in queue.available(1)
    queue.claim('a',2,{'box':(0,0,1,1),'path':'rack'})


def test_unrecoverable_released_load_does_not_retire_clear_worker_or_fake_success():
    queue=CleanupQueue(groups())
    queue.claim('a',1,{'box':(0,0,1,1),'path':'rack'})
    queue.fail('a','Load settled outside recoverable floor region')
    for released,speed,gap in [(False,0.,.2),(True,.2,.2),(True,0.,.01),(True,float('nan'),.2)]:
        with pytest.raises(ValueError):queue.quarantine_released('a',released,speed,gap)
    assert queue.items['a']['state']=='failed' and queue.slots
    queue.quarantine_released('a',True,.001,.2)
    assert queue.summary()['counts']['stored']==0
    assert queue.items['a']['state']=='blocked' and not queue.slots
    queue.claim('b',1,{'box':(0,0,1,1),'path':'rack'})


def test_exclusive_objects_and_physical_rack_footprints():
    queue=CleanupQueue(groups())
    queue.claim('a',1,{'box':(0,0,1,1),'path':'rack_a'})
    with pytest.raises(ValueError):queue.claim('a',2,{'box':(3,0,4,1),'path':'rack_b'})
    with pytest.raises(ValueError):queue.claim('b',2,{'box':(.5,.5,1.5,1.5),'path':'other_name'})
    with pytest.raises(ValueError):queue.claim('b',1,{'box':(3,0,4,1),'path':'free_rack'})
    assert queue.items['b']['state']=='pending'
    queue.complete('a',True)
    assert not queue.slot_free((.5,.5,1.5,1.5))


def test_retry_after_progress_and_never_report_false_clean():
    queue=CleanupQueue(groups())
    queue.reject(1,['a'],'No feasible route')
    assert queue.available(1)==['b']
    queue.claim('b',2,{'box':(2,0,3,1)})
    with pytest.raises(ValueError):queue.complete('b',False)
    queue.complete('b',True)
    assert queue.available(1)==['a']
    assert not queue.summary()['floor_clean']
    queue.claim('a',1,{'box':(0,0,1,1)})
    queue.fail('a','Failed lift')
    assert not queue.summary()['floor_clean']
    assert queue.summary()['counts']['failed']==1


def test_full_queue_completion_and_static_stock_protection():
    queue=CleanupQueue(groups(),static_occupancy=[(5,5,6,6)])
    assert not queue.slot_free((5,5,5.5,5.5))
    for index,key in enumerate(('a','b')):
        queue.claim(key,index,{'box':(index*2,0,index*2+1,1)})
        queue.complete(key,True)
    assert queue.summary()['floor_clean']


def test_settled_object_cannot_expand_into_stock_or_another_slot():
    queue=CleanupQueue(groups(),static_occupancy=[(5,0,6,1)])
    queue.claim('a',1,{'box':(0,0,1,1)})
    queue.claim('b',2,{'box':(2,0,3,1)})
    with pytest.raises(ValueError):queue.complete('a',True,(0,0,2.5,1))
    assert queue.items['a']['state']=='assigned'
    with pytest.raises(ValueError):queue.complete('b',True,(2,0,5.1,1))


def test_more_than_two_jobs_and_settled_footprint_are_kept_reserved():
    queue=CleanupQueue([{'id':str(i),'kind':'debris','paths':[str(i)]} for i in range(4)])
    for index in range(3):
        queue.claim(str(index),1,{'box':(index*2,0,index*2+1,1)})
        queue.complete(str(index),True,(index*2-.05,0,index*2+1.05,1))
    assert queue.summary()['counts']['stored']==3
    assert queue.available(1)==['3']
    assert not queue.summary()['floor_clean']
    assert not queue.slot_free((.9,0,1.1,1))


def test_failed_unstarted_pickup_releases_slot_for_another_robot():
    queue=CleanupQueue(groups())
    slot={'box':(0,0,1,1)}
    queue.claim('a',1,slot)
    queue.fail('a','No contact',before_manipulation=True)
    assert queue.slot_free(slot['box'])
    assert 'a' not in queue.available(1)
    assert 'a' in queue.available(2)
    queue.claim('a',2,slot)


def test_adjusting_assigned_rack_target_is_atomic_and_preserves_other_slots():
    queue=CleanupQueue(groups(),static_occupancy=[(5,0,6,1)])
    queue.claim('a',1,{'box':(0,0,1,1),'path':'rack'})
    queue.claim('b',2,{'box':(2,0,3,1),'path':'other'})
    lease=queue.items['a']['lease']
    queue.retarget('a',{'box':(.2,0,1.2,1),'path':'rack'})
    assert queue.slots[lease]['box']==(.2,0,1.2,1)
    with pytest.raises(ValueError):queue.retarget('a',{'box':(1.5,0,2.5,1),'path':'rack'})
    assert queue.slots[lease]['box']==(.2,0,1.2,1)
    assert not queue.can_retarget('a',{'box':(5,0,6,1)})
    queue.complete('a',True)
    assert not queue.can_retarget('a',{'box':(0,0,1,1)})


def test_oversized_beam_requires_longer_support_before_any_pickup():
    assert not beam_fits_support((2.60,1.12),(1.2,1.2))
    assert not beam_fits_support((1.12,2.60),(1.2,1.2))
    assert beam_fits_support((1.25,.18),(1.,1.2))


def test_recovery_requires_real_floor_settling_and_releases_stale_reservation():
    queue=CleanupQueue(groups())
    queue.claim('a',1,{'box':(0,0,1,1)})
    queue.fail('a','Navigation blocked while carrying')
    for error,speed,on_floor in ((.1,0.,True),(0.,1.,True),(0.,0.,False)):
        with pytest.raises(ValueError):queue.recovered('a',error,speed,on_floor)
    queue.recovered('a',.0001,.01,True)
    assert queue.items['a']['state']=='pending'
    assert queue.slot_free((0,0,1,1))
    assert 'a' in queue.available(2)
    assert 'a' not in queue.available(1)
    assert not queue.summary()['floor_clean']
    queue.epoch+=1  # Returning home must still give a different robot a turn.
    assert 'a' not in queue.available(1) and 'a' in queue.available(2)


def test_rolling_routes_release_long_future_corridors():
    route=[(0,0),(0,1),(4,1)]
    assert route_prefix(route,2)==[(0.,0.),(0.,1.),(1.,1.)]
    assert route_prefix(route,10)==[(0.,0.),(0.,1.),(4.,1.)]
    assert route==[(0,0),(0,1),(4,1)]


def test_oversized_items_are_blocked_not_silently_stored():
    queue=CleanupQueue(groups())
    queue.reject(1,['a'],'Oversized beam: needs longer storage')
    assert queue.summary()['counts']['blocked']==1
    assert queue.summary()['counts']['pending']==1
    assert queue.available(2)==['b']
    assert not queue.summary()['floor_clean']


def test_recovered_storage_needs_contact_support_settling_and_reserved_space():
    queue=CleanupQueue(groups())
    queue.claim('a',1,{'box':(0,0,1,1)})
    queue.fail('a','Arm retreat failed after release')
    for contact,gap,speed in ((False,0.,0.),(True,.2,0.),(True,0.,1.)):
        with pytest.raises(ValueError):queue.recovered_storage('a',contact,gap,speed,(0,0,1,1))
    assert queue.items['a']['state']=='failed'
    queue.recovered_storage('a',True,.002,.01,(0,0,1,1))
    assert queue.items['a']['state']=='stored'
    assert queue.summary()['counts']['stored']==1


def test_fifo_aisle_turns_prevent_head_on_trips_and_starvation():
    gate=FairTripGate()
    for robot in (1,2,3):gate.request(robot)
    assert not gate.enter(2)
    assert gate.enter(1)
    assert not gate.enter(2)
    gate.release(1)
    gate.request(1)
    assert not gate.enter(1)
    assert gate.enter(2)
    gate.release(3)  # Cancelling a waiting robot cannot release another's turn.
    assert gate.owner==2
    gate.release(2)
    assert gate.enter(1)


def test_independent_routes_move_together_and_crossing_route_waits():
    routes=RouteReservations()
    assert routes.reserve(1,[(0,0),(5,0)],.5,{})
    assert routes.reserve(2,[(0,3),(5,3)],.5,{})
    assert routes.peak_active==2
    assert not routes.reserve(3,[(2,-2),(2,5)],.5,{})
    assert 3 not in routes.routes
    routes.release(1)
    routes.release(2)
    assert routes.reserve(3,[(2,-2),(2,5)],.5,{})


@pytest.mark.parametrize('path',[[[5,0],[0,0]],[[2,0],[8,0]],[[2,-2],[2,2]]])
def test_route_reservations_prevent_head_on_overlap_and_crossings(path):
    routes=RouteReservations()
    assert routes.reserve(1,[(0,0),(5,0)],.6,{})
    assert not routes.reserve(2,path,.6,{})


def test_route_reservations_include_stationary_peers_and_load_radius():
    routes=RouteReservations()
    assert not routes.reserve(1,[(0,0),(5,0)],.6,{2:((3,0),.6)})
    assert routes.reserve(1,[(0,0),(5,0)],.6,{2:((3,3),.6)})
    assert not routes.reserve(2,[(0,2),(5,2)],1.6,{})


def test_long_narrow_load_uses_its_shape_near_stationary_robots():
    routes=RouteReservations()
    positions={2:((1.5,3),.2)}
    assert not routes.reserve(1,[(0,0),(0,5)],2.1,positions)
    assert routes.reserve(1,[(0,0),(0,5)],2.1,positions,footprint=(-.5,-2,.5,2))
    routes.release(1)
    assert not routes.reserve(1,[(0,0),(0,5)],2.1,{2:((.4,3),.2)},footprint=(-.5,-2,.5,2))
    with pytest.raises(ValueError):routes.reserve(1,[(float('nan'),0)],.7,{})


def test_compound_props_are_one_job_and_all_floor_categories_are_seen():
    class Prim:
        def __init__(self,name):self.name=name
        def GetName(self):return self.name
        def GetPath(self):return '/Props/'+self.name
    names=['Fallen_Carton_00','Carton_Tape_00','Tool_Wrench_Handle_00','Tool_Wrench_JawA_00',
           'Tool_Wrench_JawB_00','Rear_Left_Debris_00','Fallen_Rack_Beam_00']
    jobs,unknown=floor_groups([Prim(name) for name in names])
    assert len(jobs)==4 and not unknown
    assert sorted(len(job['paths']) for job in jobs)==[1,1,2,3]
