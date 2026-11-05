import math

from drone_agent_core.executor_core import Control, ExecutorCore
from drone_agent_core.limits import HOLD
from drone_agent_core.plans import Plan, Step, canonical_json, plan_hash

PLAN = Plan((Step("takeoff", {"altitude_m": 5.0}), Step("land")))
GOAL = (canonical_json(PLAN), plan_hash(PLAN))
FLYING = Control("AGENT", True)
GROUND = (0.0, 0.0, 0.0)


def started() -> ExecutorCore:
    core = ExecutorCore()
    assert core.start(*GOAL, FLYING, GROUND) is None
    return core


def test_idle_core_outputs_idle_and_zero():
    out = ExecutorCore().tick(GROUND, 0.0, FLYING, 0.0)
    assert out.state == "IDLE" and out.velocity == HOLD


def test_a_goal_is_refused_when_disarmed_busy_or_not_the_approved_plan():
    core = ExecutorCore()
    assert "not armed" in core.start(*GOAL, Control("IDLE", False), GROUND)
    assert core.start(*GOAL, FLYING, GROUND) is None
    assert "already running" in core.start(*GOAL, FLYING, GROUND)
    assert "hash" in ExecutorCore().start(GOAL[0], "0" * 64, FLYING, GROUND)


def test_the_first_tick_flies_and_the_velocity_is_in_the_body_frame():
    out = started().tick(GROUND, 0.0, FLYING, 0.0)
    assert out.state == "RUNNING" and out.step == 0
    assert out.velocity[2] != 0.0          # climbing: the vertical component is non-zero


def test_disarming_aborts_the_plan_and_frees_the_core():
    core = started()
    core.tick(GROUND, 0.0, FLYING, 0.0)
    out = core.tick(GROUND, 0.0, Control("AGENT", False), 1.0)
    assert out.state == "ABORTED" and out.message == "vehicle disarmed" and out.velocity == HOLD
    assert not core.busy
    assert core.start(*GOAL, FLYING, GROUND) is None


def test_another_source_pauses_and_only_the_operator_resumes():
    core = started()
    core.tick(GROUND, 0.0, FLYING, 0.0)
    assert core.tick(GROUND, 0.0, Control("TELEOP", True), 1.0).state == "PAUSED"
    assert core.tick(GROUND, 0.0, FLYING, 2.0).state == "PAUSED"       # the mux is back, still paused
    core.resume(3.0)
    assert core.tick(GROUND, 0.0, FLYING, 3.1).state == "RUNNING"

AIR = (0.0, 0.0, 10.0)
CONTINUE = Plan((Step("goto", {"x": 5.0, "y": 0.0, "altitude_m": 10.0}), Step("land")))


def test_a_plan_without_takeoff_starts_when_the_drone_is_in_the_air():
    core = ExecutorCore()
    assert core.start(canonical_json(CONTINUE), plan_hash(CONTINUE), FLYING, AIR) is None


def test_the_same_plan_is_refused_on_the_ground():
    core = ExecutorCore()
    assert "takeoff first" in core.start(canonical_json(CONTINUE), plan_hash(CONTINUE), FLYING, GROUND)


def test_a_plan_with_takeoff_is_refused_when_the_drone_is_already_up():
    assert "already airborne" in ExecutorCore().start(*GOAL, FLYING, AIR)


def test_no_position_means_no_start():
    assert "position unavailable" in ExecutorCore().start(*GOAL, FLYING, None)
