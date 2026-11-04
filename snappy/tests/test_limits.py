import math

import pytest

from drone_agent_core.limits import HOLD, ExecutorLimits, limit_velocity
from drone_agent_core.plans import PlanLimits

LIM = ExecutorLimits(max_speed_mps=2.0, max_altitude_m=20.0, geofence_radius_m=30.0, margin_m=0.5)


def test_a_safe_command_passes_unchanged():
    assert limit_velocity((5, 5, 10), (1.0, 0.5, 0.2), LIM) == ((1.0, 0.5, 0.2), None)


def test_too_fast_is_scaled_down_and_keeps_its_direction():
    v, breach = limit_velocity((0, 0, 10), (3.0, 4.0, 0.0), LIM)       # speed 5, limit 2
    assert breach is None
    assert v == pytest.approx((1.2, 1.6, 0.0))


def test_a_small_overshoot_inside_the_margin_is_not_a_breach():
    assert limit_velocity((0, 0, 20.3), (0.0, 0.0, 0.0), LIM)[1] is None
    assert limit_velocity((30.3, 0, 5), (0.0, 0.0, 0.0), LIM)[1] is None


def test_altitude_beyond_the_margin_holds_and_gives_a_reason():
    v, breach = limit_velocity((0, 0, 20.6), (1.0, 0.0, 1.0), LIM)
    assert v == HOLD
    assert "altitude" in breach


def test_leaving_the_geofence_holds_and_gives_a_reason():
    v, breach = limit_velocity((31, 0, 5), (1.0, 0.0, 0.0), LIM)
    assert v == HOLD
    assert "geofence" in breach


def test_non_finite_input_holds():
    for position, velocity in (((math.nan, 0, 5), (0.0, 0.0, 0.0)), ((0, 0, 5), (math.inf, 0.0, 0.0))):
        v, breach = limit_velocity(position, velocity, LIM)
        assert v == HOLD and breach is not None


def test_the_compilers_limits_are_inside_the_executors():
    plan, executor = PlanLimits(), ExecutorLimits()
    assert plan.max_altitude_m <= executor.max_altitude_m
    assert plan.geofence_radius_m <= executor.geofence_radius_m


def test_limits_reject_bad_values():
    with pytest.raises(ValueError):
        ExecutorLimits(max_altitude_m=0)
    with pytest.raises(ValueError):
        ExecutorLimits(margin_m=-1)