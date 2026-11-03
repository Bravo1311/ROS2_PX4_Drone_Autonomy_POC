import math

import pytest

from drone_agent_core.controllers import GotoConfig, goto_velocity

CFG = GotoConfig(max_speed_mps=2.0, slow_radius_m=4.0, arrive_radius_m=0.5)


def speed(v):
    return math.sqrt(sum(c * c for c in v))


def test_far_away_flies_at_max_speed_toward_the_target():
    v, arrived = goto_velocity((0, 0, 0), (30, 0, 0), CFG)
    assert v == pytest.approx((2.0, 0.0, 0.0))
    assert not arrived


def test_inside_the_slow_radius_speed_is_proportional_to_distance():
    v, _ = goto_velocity((0, 0, 0), (0, 2, 0), CFG)      # half the slow radius
    assert v == pytest.approx((0.0, 1.0, 0.0))


def test_speed_never_exceeds_the_maximum_and_falls_as_it_gets_close():
    speeds = [speed(goto_velocity((0, 0, 0), (d, 0, 0), CFG)[0]) for d in (100, 10, 4, 3, 2, 1, 0.6)]
    assert max(speeds) <= 2.0 + 1e-9
    assert speeds[2:] == sorted(speeds[2:], reverse=True)
    assert speeds[-1] < speeds[2]


def test_arrival_stops_the_drone():
    assert goto_velocity((0, 0, 0), (0.3, 0, 0), CFG) == ((0.0, 0.0, 0.0), True)


def test_vertical_target_commands_only_vertical_velocity():
    v, _ = goto_velocity((0, 0, 0), (0, 0, 10), CFG)
    assert v == pytest.approx((0.0, 0.0, 2.0))


def test_non_finite_input_holds():
    assert goto_velocity((math.nan, 0, 0), (1, 0, 0), CFG) == ((0.0, 0.0, 0.0), False)


def test_config_rejects_a_slow_radius_smaller_than_the_arrival_radius():
    with pytest.raises(ValueError):
        GotoConfig(slow_radius_m=0.2, arrive_radius_m=0.3)