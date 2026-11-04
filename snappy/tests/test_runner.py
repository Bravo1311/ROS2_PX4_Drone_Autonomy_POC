import pytest

from drone_agent_core.plans import Plan, Step
from drone_agent_core.runner import RunState, Runner


def plan(*steps):
    return Plan(tuple(steps))


def test_the_runner_starts_running_on_a_good_plan():
    r = Runner(plan(Step("hold", {"seconds": 2})))
    assert r.state is RunState.RUNNING


def test_the_runner_refuses_a_plan_it_cannot_run():
    with pytest.raises(ValueError):
        Runner(plan(Step("land")))                       # no land skill yet
    with pytest.raises(ValueError):
        Runner(plan())