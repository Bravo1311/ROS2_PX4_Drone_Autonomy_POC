from drone_agent_core.guard import Decision, Guard, Risk, ToolSpec, Verdict
from drone_agent_core.profile import Policy

BATTERY = ToolSpec("get_battery", Risk.OBSERVE, "/fmu/out/battery_status")
TAKEOFF = ToolSpec("run_mission", Risk.MOTION)
RAW_SETPOINT = ToolSpec("raw", Risk.OBSERVE, "/fmu/in/trajectory_setpoint")
DENY_LIST = ("/fmu/in/*", "*cmd_vel*")


def make_guard(autonomy="supervised", armed=False):
    return Guard(Policy(start_armed=armed, autonomy=autonomy), DENY_LIST)


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