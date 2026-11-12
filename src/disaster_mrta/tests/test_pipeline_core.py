from disaster_mrta.pipeline_core import FleetState, FailureTracker, baseline_actions, observation
from disaster_mrta.perception_model import vision_result, CLASSES
import pytest


def fleet():
    return FleetState(['robot_1','robot_2'],[dict(task_id='a',task_type='area_search',x=1.,y=0.,priority=.8,required_capability='camera')])


def status(rid='robot_1',sequence=1,**kwargs):
    return dict(robot_id=rid,sequence=sequence,boot_id='boot_1',x=0.,y=0.,battery=100.,capabilities=['camera'],
                healthy=True,communication_available=True)|kwargs


def test_feasibility_battery_distance_and_communication():
    f=fleet()
    assert f.mask()==[[True,False],[True,False]]
    f.status(status(battery=19.9))
    assert not f.mask()[0][1]
    f.status(status(sequence=2,battery=20.))
    assert f.mask()[0][1]
    f.status(status(sequence=3,x=52.))
    assert not f.mask()[0][1]
    f.status(status(sequence=4,communication_available=False))
    assert not f.mask()[0][1]


def test_duplicate_assignments_and_stale_completion_after_failure():
    f=fleet()
    f.status(status())
    f.status(status('robot_2'))
    assert baseline_actions(f)==[1,0]
    old=f.commit('robot_1','a')
    assert f.commit('robot_2','a') is None
    cancel=f.release_robot('robot_1','lost',1.)
    assert cancel['assignment_id']==old['assignment_id']
    new=f.commit('robot_2','a')
    assert new['assignment_id']!=old['assignment_id']
    assert not f.feedback(dict(old,status='completed'))
    assert f.feedback(dict(new,status='in_progress'))
    assert f.feedback(dict(new,status='completed'))
    assert not f.feedback(dict(new,status='completed'))


def test_exactly_three_missed_updates_and_recovery():
    f=fleet()
    m=FailureTracker(list(f.robots))
    f.status(status())
    assert m.tick(f.robots)==[]
    assert m.tick(f.robots)==[]
    assert m.tick(f.robots)==[]
    assert m.tick(f.robots)==['robot_1']
    assert m.tick(f.robots)==[]
    f.status(status(sequence=2))
    assert m.tick(f.robots)==[]


def test_restart_sequences_and_old_session_rejection():
    f=fleet()
    f.status(status(sequence=100))
    assert f.status(status(sequence=0,boot_id='boot_2'))
    assert not f.status(status(sequence=101,boot_id='boot_1'))
    assert f.robots['robot_1'].boot_id=='boot_2'


def test_human_damage_is_only_an_area_clue_and_raises_priority():
    f=fleet()
    v=vision_result([0,0,.9,0,.1,0],list(CLASSES),'a','robot_1')
    f.vision(v)
    assert f.tasks['a'].priority==.9
    assert 'no specific person' in v['interpretation']
    assert 'victim_location' not in v
    assert len(observation(f,8)[0])==69


def test_invalid_prediction_and_capacity_fail_closed():
    with pytest.raises(ValueError):
        vision_result([float('nan')]*6,list(CLASSES),'a','robot_1')
    with pytest.raises(ValueError):
        observation(fleet(),0)


def test_invalid_status_does_not_retire_valid_boot():
    f=fleet()
    f.status(status())
    with pytest.raises(ValueError):
        f.status(status(sequence=2,boot_id='bad',x=float('nan')))
    assert f.robots['robot_1'].boot_id=='boot_1'
    with pytest.raises(ValueError):
        f.status(status(sequence=2,healthy='false'))


def test_completion_requires_execution_start_and_missing_qmix_fails_clearly():
    f=fleet()
    f.status(status())
    assignment=f.commit('robot_1','a')
    assert not f.feedback(dict(assignment,status='completed'))
    from disaster_mrta.qmix_policy import QmixPolicy
    with pytest.raises(ValueError,match='trained checkpoint'):
        QmixPolicy('',2,8)


def test_unsafe_delivery_route_revokes_lease_without_failing_robot():
    f=FleetState(['robot_1'],[dict(task_id='d',task_type='supply_delivery',x=1.,y=0.,priority=.8,
                                required_capability='delivery',route_safe=True)])
    f.status(status(capabilities=['delivery']))
    assignment=f.commit('robot_1','d')
    f.tasks['d'].route_safe=False
    f.cancel_task('d','unsafe route')
    assert f.robots['robot_1'].online
    assert f.robots['robot_1'].current_task is None
    assert not f.mask()[0][1]
    assert not f.feedback(dict(assignment,status='completed'))


def test_checkpoint_metadata_rejects_wrong_labels_or_preprocessing(tmp_path):
    import json
    from disaster_mrta.perception_model import EfficientNetModel,CLASSES
    model=tmp_path/'model.keras'
    model.write_bytes(b'not loaded: metadata validation must fail first')
    metadata=model.with_suffix('.metadata.json')
    metadata.write_text(json.dumps({'class_names':list(reversed(CLASSES)),
                                   'input_range':'0_255','model_output':'probabilities'}))
    with pytest.raises(ValueError,match='class order'):
        EfficientNetModel(str(model),list(CLASSES))
    metadata.write_text(json.dumps({'class_names':list(CLASSES),
                                   'input_range':'0_1','model_output':'probabilities'}))
    with pytest.raises(ValueError,match='preprocessing'):
        EfficientNetModel(str(model),list(CLASSES))
