from pathlib import Path

import pytest

from disaster_mrta.config_loader import load_config
from disaster_mrta.tasks import Task, action_mask, area_tasks, assignment_features


def fleet():
    return load_config(Path(__file__).parents[1] / 'config' / 'robots.yaml')['robots']


def test_human_damage_is_search_clue_for_both_robot_types():
    tasks = area_tasks('area', (2, 3), {'Human_Damage': .9})
    assert len(tasks) == 1
    assert tasks[0].task_type == 'area_search'
    assert 'no person localized' in tasks[0].evidence
    assert all(action_mask(robot, tasks) == [True, True] for robot in fleet())
    assert len(tasks[0].visual_risk_scores) == 6


def test_hazard_inspection_requires_ground_robot_and_matching_capability():
    task = area_tasks('area', (2, 3), {'Fire_Disaster': .9})[1]
    assert task.task_type == 'hazard_inspection'
    assert [action_mask(r, [task])[1] for r in fleet()] == [False, True, False, False]
    assert task.priority > area_tasks('area', (2, 3), {'Fire_Disaster': .6})[1].priority


def test_delivery_requires_capability_safe_route_and_sufficient_battery():
    task = Task('delivery', 'supply_delivery', (12, 0), .8, 'supply_delivery', 15)
    robot = fleet()[3]
    assert action_mask(robot, [task]) == [True, False]
    assert action_mask(robot, [task], {'delivery': False}) == [True, False]
    assert action_mask(robot, [task], {'delivery': True}) == [True, True]
    assert not action_mask(fleet()[0], [task], {'delivery': True})[1]
    robot['battery'] = 10
    assert not action_mask(robot, [task], {'delivery': True})[1]


@pytest.mark.parametrize('field,value', [('available', False), ('healthy', False),
    ('communication_available', False), ('current_task', 'busy')])
def test_unavailable_robots_can_only_idle(field, value):
    robot = fleet()[1]
    robot[field] = value
    assert action_mask(robot, area_tasks('area', (0, 0), {})) == [True, False]


def test_completed_tasks_and_unsafe_routes_are_masked():
    task = area_tasks('area', (0, 0), {})[0]
    assert not assignment_features(fleet()[1], task, route_safe=False)['feasible']
    task.status = 'completed'
    assert action_mask(fleet()[1], [task]) == [True, False]


def test_invalid_visual_scores_are_rejected():
    with pytest.raises(ValueError):
        area_tasks('area', (0, 0), {'Human_Damage': float('nan')})
