from drone_agent_core.plans import Step, check_step, Plan, PlanLimits, compile_plan, plan_hash, canonical_json, render_card

import pytest


def plan(*steps):
    return Plan(tuple(steps))


GOOD = plan(
    Step("takeoff", {"altitude_m": 5}),
    Step("goto", {"x": 10.0, "y": 0.0, "altitude_m": 5}),
    Step("hold", {"seconds": 3}),
    Step("land"),
)


def test_valid_step_passes():
    assert check_step(Step("takeoff", {"altitude_m": 5.0})) is None
    assert check_step(Step("takeoff", {"altitude": 5.0})) is not None

def test_unknown_skill_is_rejected():
    assert check_step(Step("barrel_roll")) is not None

def test_missing_argument_is_rejected():
    assert check_step(Step("goto", {"x": 1.0, "y": 2.0})) is not None


def test_land_takes_no_arguments():
    assert check_step(Step("land")) is None
    assert check_step(Step("land", {"x": 1.0})) is not None

def test_good_plan_compiles():
    assert compile_plan(GOOD) == []


def test_empty_plan_is_rejected():
    assert compile_plan(plan()) != []


def test_goto_before_takeoff_is_rejected():
    errors = compile_plan(plan(Step("goto", {"x": 1.0, "y": 1.0, "altitude_m": 5}), Step("land")))
    assert any("takeoff first" in e for e in errors)


def test_plan_must_end_with_land():
    errors = compile_plan(plan(Step("takeoff", {"altitude_m": 5})))
    assert "plan must end with land" in errors


def test_altitude_above_limit_is_rejected():
    errors = compile_plan(plan(Step("takeoff", {"altitude_m": 500}), Step("land")))
    assert len(errors) == 1


def test_nan_target_is_outside_the_geofence():
    nan = float("nan")
    errors = compile_plan(plan(Step("takeoff", {"altitude_m": 5}),
                               Step("goto", {"x": nan, "y": 0.0, "altitude_m": 5}),
                               Step("land")))
    assert len(errors) == 1


def test_every_problem_is_reported():
    errors = compile_plan(plan(Step("barrel_roll"), Step("hold", {"seconds": 3}), Step("land")))
    assert len(errors) == 3


def test_plan_can_start_while_airborne():
    p = plan(Step("hold", {"seconds": 3}), Step("land"))
    assert compile_plan(p) != []
    assert compile_plan(p, airborne=True) == []

def test_canonical_json_is_pinned():
    p = plan(Step("takeoff", {"altitude_m": 5}))
    assert canonical_json(p) == '{"steps":[{"args":{"altitude_m":5.0},"skill":"takeoff"}]}'


def test_int_and_float_give_the_same_hash():
    assert plan_hash(plan(Step("takeoff", {"altitude_m": 5}))) == plan_hash(plan(Step("takeoff", {"altitude_m": 5.0})))


def test_argument_order_does_not_change_the_hash():
    a = plan(Step("goto", {"x": 1.0, "y": 2.0, "altitude_m": 5.0}))
    b = plan(Step("goto", {"altitude_m": 5.0, "y": 2.0, "x": 1.0}))
    assert plan_hash(a) == plan_hash(b)


def test_different_plans_have_different_hashes():
    assert plan_hash(plan(Step("takeoff", {"altitude_m": 5}))) != plan_hash(plan(Step("takeoff", {"altitude_m": 6})))
    assert plan_hash(plan(Step("land"), Step("hold", {"seconds": 1}))) != plan_hash(plan(Step("hold", {"seconds": 1}), Step("land")))


def test_hash_is_64_hex_characters():
    h = plan_hash(GOOD)
    assert len(h) == 64 and all(c in "0123456789abcdef" for c in h)

def test_card_lists_every_step_and_the_short_hash():
    card = render_card(GOOD)
    assert plan_hash(GOOD)[:12] in card
    assert "1. takeoff to 5 m" in card
    assert "4. land" in card


def test_card_summary_comes_from_the_steps():
    card = render_card(GOOD)
    assert "highest 5 m (limit 30)" in card
    assert "farthest 10 m from start (limit 50)" in card
    assert "holding 3 s" in card


def test_card_refuses_a_plan_that_does_not_compile():
    with pytest.raises(ValueError):
        render_card(plan(Step("goto", {"x": 1.0, "y": 1.0, "altitude_m": 5.0})))