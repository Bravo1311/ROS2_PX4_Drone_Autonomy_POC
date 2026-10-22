from drone_agent_core.plans import Step, check_step, Plan, PlanLimits, compile_plan


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