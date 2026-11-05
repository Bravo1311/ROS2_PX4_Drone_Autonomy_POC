import math
import pytest
from drone_agent_core.frames import world_to_body

def test_facing_north_east_is_right_and_north_is_forward():
    assert world_to_body((1.0, 0.0, 0.0), 0.0) == pytest.approx((0.0, 1.0, 0.0))
    assert world_to_body((0.0, 1.0, 0.0), 0.0) == pytest.approx((1.0, 0.0, 0.0))

def test_facing_east_east_is_forward_and_north_is_left():
    assert world_to_body((1.0, 0.0, 0.0), math.pi / 2) == pytest.approx((1.0, 0.0, 0.0))
    assert world_to_body((0.0, 1.0, 0.0), math.pi / 2) == pytest.approx((0.0, -1.0, 0.0))

def test_speed_is_unchanged_by_the_rotation():
    body = world_to_body((3.0, 4.0, 0.0), 1.1)
    assert math.hypot(body[0], body[1]) == pytest.approx(5.0)