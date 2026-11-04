import math

import pytest

from drone_agent_core.limits import ExecutorLimits
from drone_agent_core.plans import Plan, Step
from drone_agent_core.runner import RunState, Runner

MISSION = Plan((
    Step("takeoff", {"altitude_m": 5}),
    Step("goto", {"x": 10.0, "y": 0.0, "altitude_m": 5.0}),
    Step("hold", {"seconds": 2}),
    Step("land"),
))


def fly(plan, start, dt=0.1, max_time_s=300.0, **runner_kwargs):
    """A tiny fake drone: it does exactly the velocity it is told. Returns every (position, result)."""
    runner = Runner(plan, **runner_kwargs)
    position, now, trail = start, 0.0, []
    while now < max_time_s:
        result = runner.tick(position, now)
        trail.append((position, result))
        if result.state is not RunState.RUNNING:
            return trail
        position = tuple(p + v * dt for p, v in zip(position, result.velocity))
        now += dt
    raise AssertionError("the run did not finish in time")


def test_a_whole_mission_flies_to_the_end():
    trail = fly(MISSION, (0.0, 0.0, 0.0))
    last_position, last = trail[-1]
    assert last.state is RunState.DONE and last.step == 4
    assert last_position[0] == pytest.approx(10.0, abs=0.3)
    assert last_position[1] == pytest.approx(0.0, abs=0.01)
    assert last_position[2] <= 0.15
    steps = [r.step for _, r in trail]
    assert steps == sorted(steps)
    assert max(math.hypot(*r.velocity) for _, r in trail) <= 1.5 + 1e-9
    assert max(p[2] for p, _ in trail) <= 5.3


def test_a_goto_beyond_the_geofence_aborts_when_the_line_is_crossed():
    far = Plan((Step("goto", {"x": 100.0, "y": 0.0, "altitude_m": 5.0}),))
    trail = fly(far, (0.0, 0.0, 5.0), limits=ExecutorLimits(geofence_radius_m=55.0))
    last_position, last = trail[-1]
    assert last.state is RunState.ABORTED and "geofence" in last.message
    assert 55.5 < last_position[0] < 56.0
