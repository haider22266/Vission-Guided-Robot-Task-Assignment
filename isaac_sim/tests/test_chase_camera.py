import math
import sys
from pathlib import Path

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from camera_tour import chase_pose


def test_chase_is_low_and_behind_eastbound_robot():
    eye,target=chase_pose({'x':2.,'y':3.,'z':.1,'heading_degrees':0.})
    assert eye==(.8,3.,.45)
    assert target==(2.25,3.,.28)


def test_chase_turns_with_robot_and_honors_distance():
    eye,target=chase_pose({'x':2.,'y':3.,'z':.1,'heading_degrees':90.},.35,.8)
    assert math.isclose(eye[0],2.) and math.isclose(eye[1],2.2)
    assert eye[2]==.35 and target[1]>3
