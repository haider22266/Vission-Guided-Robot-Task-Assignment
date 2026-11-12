import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from exploration import patrol_pose, warehouse_patrols


def test_constant_speed_turn_and_wrap():
    points = [(0,0,0), (4,0,0), (4,3,0)]
    assert patrol_pose(points, 2, 1) == ((2.,0.,0.), 0.)
    assert patrol_pose(points, 2, 2.5) == ((4.,1.,0.), 90.)
    assert patrol_pose(points, 2, 6) == ((0.,0.,0.), 0.)


def test_backtracking_faces_direction_of_travel():
    position, yaw = patrol_pose([(0,0,0),(0,4,0)], 1, 5)
    assert position == (0.,3.,0.)
    assert yaw == -90


def test_warehouse_routes_stay_in_aisles_and_avoid_front_debris():
    for route, speed in warehouse_patrols():
        for step in range(1000):
            (x,y,z), yaw = patrol_pose(route, speed, step*.5)
            assert math.isfinite(yaw)
            assert -9 <= y <= 14
            # Rack spans x around -10, 0, 10 and y -5..11.
            if -5 <= y <= 11:
                assert 1.3 < abs(x) < 8.7
            if -7 <= y <= -5.5 and z < 1:
                assert abs(x) >= 7.7


def test_zero_length_patrol_rejected():
    with pytest.raises(ValueError):
        patrol_pose([(0,0,0),(0,0,0)], 1, 0)
