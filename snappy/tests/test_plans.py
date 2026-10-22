from drone_agent_core.plans import Step, check_step

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