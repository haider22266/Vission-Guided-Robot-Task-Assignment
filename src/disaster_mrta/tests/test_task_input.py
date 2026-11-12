import json
from pathlib import Path

import pytest

from disaster_mrta.tasks import load_tasks, assignment_features
from disaster_mrta.config_loader import load_config


def test_user_task_format_keeps_coordinates_and_ownership():
    path=Path(__file__).resolve().parents[3]/'isaac_sim'/'config'/'tasks.json'
    tasks=load_tasks(path)
    assert [t.task_id for t in tasks]==['0','1','2']
    assert [t.location for t in tasks]==[(-3.,2.),(3.,2.),(0.,-3.)]
    assert all(t.location_policy=='fixed' and t.assigned_robot is None for t in tasks)
    robots=load_config(Path(__file__).parents[1]/'config'/'robots.yaml')['robots']
    assert assignment_features(robots[0],tasks[0])['feasible']
    assert assignment_features(robots[1],tasks[1])['feasible']
    assert assignment_features(robots[3],tasks[2],True)['feasible']
    assert not assignment_features(robots[0],tasks[2],True)['feasible']


def test_duplicate_user_ids_rejected(tmp_path):
    path=tmp_path/'tasks.json'
    row={'id':0,'type':'area_search','x':0,'y':0,'priority':.8,'required_capability':'camera'}
    path.write_text(json.dumps([row,row]))
    with pytest.raises(ValueError,match='Duplicate'):
        load_tasks(path)
