from drone_agent_core.guard import Decision, Guard, Risk, ToolSpec, Verdict
from drone_agent_core.profile import Policy

BATTERY = ToolSpec("get_battery", Risk.OBSERVE, "/fmu/out/battery_status")
TAKEOFF = ToolSpec("run_mission", Risk.MOTION)
RAW_SETPOINT = ToolSpec("raw", Risk.OBSERVE, "/fmu/in/trajectory_setpoint")
DENY_LIST = ("/fmu/in/*", "*cmd_vel*")
ARGS1 = {"a": 1, "b": 2}
ARGS2 = {"a": 3, "b": 4}

POSE = ToolSpec("get_pose", Risk.OBSERVE, "/fmu/out/vehicle_local_position")


def make_guard(autonomy="supervised", armed=False, max_calls_per_turn=2, repeat_limit=99):
    return Guard(Policy(
            start_armed=armed,
            autonomy=autonomy,
            max_calls_per_turn=max_calls_per_turn,
            repeat_limit=repeat_limit,
        ), DENY_LIST)


def make_repeat_guard():
    return make_guard(max_calls_per_turn=10, repeat_limit=3)


def test_observe_tool_is_allowed_while_disarmed():
    assert make_guard().decide(BATTERY).verdict == Verdict.ALLOW


def test_motion_while_disarmed_is_denied():
    decision = make_guard().decide(TAKEOFF)

    assert decision.verdict == Verdict.DENY
    assert decision.rule == "disarmed"


def test_motion_when_armed_needs_approval():
    decision = make_guard(armed=True).decide(TAKEOFF)

    assert decision.verdict == Verdict.NEED_APPROVAL


def test_arm_command_changes_the_decision():
    guard = make_guard()
    guard.arm()

    assert guard.decide(TAKEOFF).verdict == Verdict.NEED_APPROVAL


def test_observe_mode_denies_motion_even_when_armed():
    decision = make_guard(autonomy="observe", armed=True).decide(TAKEOFF)

    assert decision.verdict == Verdict.DENY
    assert decision.rule == "observe_only"


def test_hard_deny_applies_even_to_observe_risk_and_when_armed():
    decision = make_guard(armed=True).decide(RAW_SETPOINT)

    assert decision.verdict == Verdict.DENY
    assert decision.rule == "hard_deny"


def test_rule_order_observe_only_comes_before_disarmed():
    decision = make_guard(autonomy="observe", armed=False).decide(TAKEOFF)

    assert decision.rule == "observe_only"

def test_max_calls_per_turn_limit():
    guard = make_guard()
    assert guard.decide(BATTERY).verdict == Verdict.ALLOW
    assert guard.decide(BATTERY).verdict == Verdict.ALLOW
    decision = guard.decide(BATTERY)
    assert decision.verdict == Verdict.DENY and decision.rule == "budget"

def test_max_calls_per_turn_limit_with_denied_calls():
    guard = make_guard()
    assert guard.decide(BATTERY).verdict == Verdict.ALLOW
    assert guard.decide(RAW_SETPOINT).verdict == Verdict.DENY
    decision = guard.decide(BATTERY)
    assert decision.verdict == Verdict.DENY and decision.rule == "budget"

def test_budget_resets_on_new_turn():
    guard = make_guard()
    assert guard.decide(BATTERY).verdict == Verdict.ALLOW
    assert guard.decide(BATTERY).verdict == Verdict.ALLOW
    decision = guard.decide(BATTERY)
    assert decision.verdict == Verdict.DENY and decision.rule == "budget"
    guard.begin_turn()
    assert guard.decide(BATTERY).verdict == Verdict.ALLOW

def test_begin_turn_does_not_disarm():
    guard = make_guard(armed=True)
    assert guard.decide(BATTERY).verdict == Verdict.ALLOW
    assert guard.decide(BATTERY).verdict == Verdict.ALLOW
    decision = guard.decide(BATTERY)
    assert decision.verdict == Verdict.DENY and decision.rule == "budget"
    guard.begin_turn()
    assert guard.armed is True

def test_fourth_identical_call_is_denied_as_repeat():
    guard = make_repeat_guard()
    for _ in range(3):
        assert guard.decide(BATTERY, ARGS1).verdict == Verdict.ALLOW
    decision = guard.decide(BATTERY, ARGS1)
    assert decision.verdict == Verdict.DENY and decision.rule == "repeat"

def test_same_tool_different_args_is_not_a_repeat():
    guard = make_repeat_guard()
    assert guard.decide(BATTERY, ARGS1).verdict == Verdict.ALLOW
    assert guard.decide(BATTERY, ARGS2).verdict == Verdict.ALLOW
    assert guard.decide(BATTERY, ARGS1).verdict == Verdict.ALLOW

def test_different_call_resets_the_run():
    guard = make_repeat_guard()
    for tool in (BATTERY, BATTERY, BATTERY, POSE, BATTERY, BATTERY, BATTERY):
        assert guard.decide(tool).verdict == Verdict.ALLOW

def test_begin_turn_clears_the_repeat_run():
    guard = make_repeat_guard()
    for _ in range(3):
        assert guard.decide(BATTERY).verdict == Verdict.ALLOW
    guard.begin_turn()
    assert guard.decide(BATTERY).verdict == Verdict.ALLOW

def test_no_args_and_empty_args_are_the_same_call():
    guard = make_repeat_guard()
    assert guard.decide(BATTERY).verdict == Verdict.ALLOW
    assert guard.decide(BATTERY, {}).verdict == Verdict.ALLOW
    assert guard.decide(BATTERY, None).verdict == Verdict.ALLOW
    assert guard.decide(BATTERY).rule == "repeat"

def test_args_mutated_after_the_call_do_not_change_the_record():
    guard = make_repeat_guard()
    args = {"a": 1}
    assert guard.decide(BATTERY, args).verdict == Verdict.ALLOW
    args["a"] = 2
    for _ in range(3):
        assert guard.decide(BATTERY, {"a": 2}).verdict == Verdict.ALLOW
    assert guard.decide(BATTERY, {"a": 2}).rule == "repeat"

def test_repeat_limit_of_one_allows_each_call_once():
    guard = make_guard(max_calls_per_turn=10, repeat_limit=1)
    assert guard.decide(BATTERY).verdict == Verdict.ALLOW
    assert guard.decide(BATTERY).rule == "repeat"

def test_invalid_calls_are_denied_then_capped():
    guard = make_guard(max_calls_per_turn=10)
    for _ in range(3):
        d = guard.record_invalid("missing argument")
        assert d.verdict == Verdict.DENY and d.rule == "invalid"
    d = guard.record_invalid("missing argument")
    assert d.verdict == Verdict.DENY and d.rule == "invalid_cap"

def test_invalid_calls_use_up_the_budget():
    guard = make_guard(max_calls_per_turn=2)
    guard.record_invalid("bad")
    guard.record_invalid("bad")
    assert guard.decide(BATTERY).rule == "budget"

def test_begin_turn_resets_the_invalid_count():
    guard = make_guard(max_calls_per_turn=10)
    for _ in range(4):
        guard.record_invalid("bad")
    guard.begin_turn()
    assert guard.record_invalid("bad").rule == "invalid"
