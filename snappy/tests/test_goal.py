from drone_agent_core.goal import accept_goal
from drone_agent_core.plans import Plan, Step, canonical_json, plan_hash

PLAN = Plan((Step("takeoff", {"altitude_m": 5.0}), Step("hold", {"seconds": 3.0}), Step("land")))
PLAN2 = Plan((Step("takeoff", {"altitude_m": 6.0}), Step("hold", {"seconds": 3.0}), Step("land")))


def test_the_approved_plan_is_accepted():
    runner, error = accept_goal(canonical_json(PLAN), plan_hash(PLAN))
    assert runner is not None and error is None


def test_text_that_is_not_json_is_rejected():
    runner, error = accept_goal("takeoff", plan_hash(PLAN))
    assert runner is None and "not valid JSON" in error


def test_json_without_steps_is_rejected():
    runner, error = accept_goal('{"plan": []}', plan_hash(PLAN))
    assert runner is None and "steps" in error


def test_a_plan_that_does_not_compile_is_rejected():
    bad = '{"steps":[{"skill":"barrel_roll","args":{}}]}'
    runner, error = accept_goal(bad, "0" * 64)
    assert runner is None and "barrel_roll" in error


def test_nan_altitude_is_rejected_before_it_can_be_hashed():
    bad = '{"steps":[{"skill":"takeoff","args":{"altitude_m":NaN}},{"skill":"land"}]}'
    runner, error = accept_goal(bad, "0" * 64)
    assert runner is None and error.startswith("goal rejected")
    
def test_a_plan_that_differs_from_the_approved_one_is_rejected():
    runner, error = accept_goal(canonical_json(PLAN), plan_hash(PLAN2))
    assert runner is None and "hash does not match" in error