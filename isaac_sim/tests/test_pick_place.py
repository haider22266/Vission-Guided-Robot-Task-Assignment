import ast
import sys
from pathlib import Path

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from pick_place_tasks import Item, TaskBroker, verified_contact


def broker():
    return TaskBroker([Item('carton_a','carton',0,.044,.05),Item('carton_b','carton',0,.044,.05)],
                      {f'slot_{i}':{'lane':0,'position':(i,0,0)} for i in range(3)})


def contact():
    return dict(center_error=.0005,left_gap=.0005,right_gap=.0005,orientation_error=.001,stable_frames=5)


def test_no_duplicate_object_or_slot_assignments():
    b=broker()
    a=b.claim('robot_a','carton',0)
    other=b.claim('robot_b','carton',0)
    assert a['object']!=other['object'] and a['slot']!=other['slot']
    assert b.claim('robot_c','carton',0) is None
    assert b.claim('robot_a','carton',0) is a


def test_visible_gap_prevents_pickup_and_completion():
    b=broker()
    task=b.claim('robot','carton',0)
    errors=contact()
    errors['left_gap']=.03
    assert not b.grasp('robot',errors)
    assert not b.complete('robot',0.,.01)
    assert b.items[task['object']].state=='available'


def test_verified_pick_place_then_another_task():
    b=broker()
    task=b.claim('robot','carton',0)
    assert b.grasp('robot',contact())
    assert not b.complete('robot',.02,.01)
    assert not b.complete('robot',.001,0.)
    assert b.complete('robot',.001,.01)
    assert b.history[-1]['status']=='completed'
    assert b.claim('robot','carton',0)['object']!=task['object']


def test_unstable_contact_is_not_enough():
    errors=contact()
    errors['stable_frames']=1
    assert not verified_contact(**errors)


def test_continuous_relocation_keeps_old_slot_until_verified_pickup():
    b=broker()
    for _ in range(2):
        b.claim('robot','carton',0)
        assert b.grasp('robot',contact())
        assert b.complete('robot',.001,.01)
    task=b.claim('robot','carton',0)
    old_slot=task['source_slot']
    assert old_slot in b.occupied
    assert task['slot']!=old_slot
    assert b.grasp('robot',contact())
    assert old_slot not in b.occupied
    assert b.complete('robot',.001,.01)


def test_fixed_camera_has_no_pose_updates_inside_loop():
    source=Path(__file__).resolve().parents[1]/'five_robot_pick_place.py'
    tree=ast.parse(source.read_text())
    calls=[node for node in ast.walk(tree) if isinstance(node,ast.Call)
           and isinstance(node.func,ast.Name) and node.func.id=='set_camera_view']
    assert len(calls)==1
    for loop in (n for n in ast.walk(tree) if isinstance(n,(ast.For,ast.While))):
        assert not any(call is calls[0] for call in ast.walk(loop))
