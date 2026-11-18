import math
from pathlib import Path
import sys
import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from small_cleanup_scene import small_box_positions


@pytest.mark.parametrize('count',[5,10,15,20])
def test_small_props_are_unique_and_spaced_for_five_workers(count):
    points=small_box_positions(count)
    assert len(points)==count and len(set(points))==count
    assert min(math.dist(a,b) for i,a in enumerate(points) for b in points[i+1:])>=2.4
    assert all(x!=0 for x,y in points)  # Keep the middle rack line empty.


def test_small_demo_rejects_unsupported_counts():
    with pytest.raises(ValueError):small_box_positions(88)
