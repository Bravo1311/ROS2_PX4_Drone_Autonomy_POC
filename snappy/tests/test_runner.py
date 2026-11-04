import pytest

from drone_agent_core.plans import Plan, Step
from drone_agent_core.runner import RunState, Runner
from drone_agent_core.controllers import GotoConfig
from drone_agent_core.limits import HOLD, ExecutorLimits


def plan(*steps):
    return Plan(tuple(steps))


def test_the_runner_starts_running_on_a_good_plan():
    r = Runner(plan(Step("hold", {"seconds": 2})))
    assert r.state is RunState.RUNNING


def test_the_runner_refuses_a_plan_it_cannot_run():
    with pytest.raises(ValueError):
        Runner(plan(Step("landing")))                       # no land skill yet
    with pytest.raises(ValueError):
        Runner(plan())

def test_a_hold_step_finishes_after_its_time_and_the_plan_is_done():
    r = Runner(plan(Step("hold", {"seconds": 2})))    
    assert r.tick((0, 0, 5), 0.0).state is RunState.RUNNING
    assert r.tick((0, 0, 5), 1.9).state is RunState.RUNNING
    result = r.tick((0, 0, 5), 2.0)
    assert result.state is RunState.DONE and result.step == 1


def test_steps_run_in_order_and_the_next_timer_starts_on_its_first_tick():
    r = Runner(plan(Step("takeoff", {"altitude_m": 5}), Step("hold", {"seconds": 1})))
    first = r.tick((0, 0, 0), 0.0)
    assert first.step == 0 and first.velocity[2] > 0
    reached = r.tick((0, 0, 4.9), 1.0)                  # takeoff done, hold not started yet
    assert reached.step == 1 and reached.state is RunState.RUNNING
    assert r.tick((0, 0, 5.0), 1.1).state is RunState.RUNNING      # hold's clock starts here
    assert r.tick((0, 0, 5.0), 2.1).state is RunState.DONE


def test_a_limit_breach_aborts_the_plan_and_it_stays_aborted():
    r = Runner(plan(Step("hold", {"seconds": 10})), limits=ExecutorLimits(max_altitude_m=20.0))
    result = r.tick((0, 0, 25.0), 0.0)
    assert result.state is RunState.ABORTED and result.velocity == HOLD
    assert "altitude" in result.message
    later = r.tick((0, 0, 5.0), 1.0)                    # back inside the limits: still aborted
    assert later.state is RunState.ABORTED and later.velocity == HOLD


def test_after_the_plan_is_done_every_tick_holds():
    r = Runner(plan(Step("hold", {"seconds": 1})))
    r.tick((0, 0, 5), 0.0)
    r.tick((0, 0, 5), 1.0)
    assert r.tick((0, 0, 5), 2.0).velocity == HOLD
    assert r.tick((0, 0, 5), 3.0).state is RunState.DONE


def test_the_executor_limit_applies_even_when_the_skill_asks_for_more_speed():
    fast = GotoConfig(max_speed_mps=5.0, slow_radius_m=3.0, arrive_radius_m=0.3)
    r = Runner(plan(Step("goto", {"x": 30.0, "y": 0.0, "altitude_m": 5.0})),
               limits=ExecutorLimits(max_speed_mps=2.0), goto_cfg=fast)
    v = r.tick((0, 0, 5), 0.0).velocity
    assert v == pytest.approx((2.0, 0.0, 0.0))
    
def test_a_land_step_finishes_at_the_ground():
    r = Runner(plan(Step("land")))
    assert r.tick((0, 0, 3.0), 0.0).velocity[2] < 0
    assert r.tick((0, 0, 0.1), 1.0).state is RunState.DONE