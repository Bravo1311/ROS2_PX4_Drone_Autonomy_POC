import time

from drone_agent_core.guard import Guard
from drone_agent_core.plans import SUBMIT_PLAN, canonical_json, parse_plan, plan_hash, render_card
from drone_agent_core.profile import Policy
from drone_agent_core.robot import FakeExecutor, FakeRobot, RobotUnavailable, Telemetry
from drone_agent_core.session import ModelReply, PendingPlan, Session, ToolCall
from drone_agent_core.tools import OBSERVE_TOOLS, build_registry

STEPS = [{"skill": "takeoff", "args": {"altitude_m": 5}}, {"skill": "land"}]


class OneShotModel:
    def __init__(self, steps=STEPS):
        self._replies = [ModelReply(tool_calls=(ToolCall("submit_plan", {"steps": steps}),)),
                         ModelReply("Drafted.")]

    def reply(self, messages):
        return self._replies.pop(0)


def flight_session(executor, steps=STEPS, robot=None, **policy):
    session = Session(OneShotModel(steps), robot or FakeRobot(), Guard(Policy(**policy), ("/fmu/in/*",)),
                      build_registry(OBSERVE_TOOLS + (SUBMIT_PLAN,)), executor=executor)
    session.handle_user_message("take off to 5 m and land")
    return session


def test_approve_does_not_fly_until_the_agent_is_enabled():
    executor = FakeExecutor()
    s = flight_session(executor)
    assert s.approve(s.pending.hash[:12]).startswith("NOT STARTED (disarmed)")
    assert executor.started == [] and s.pending is not None


def test_an_enabled_approve_sends_the_exact_approved_plan():
    executor = FakeExecutor()
    s = flight_session(executor)
    full_hash = s.pending.hash
    s.enable()
    assert s.approve(full_hash[:12]).startswith("APPROVED")
    assert executor.started == [(canonical_json(parse_plan(STEPS)), full_hash)]
    assert s.pending is None


def test_an_executor_refusal_is_shown_and_the_plan_stays_pending():
    s = flight_session(FakeExecutor(refuse_with="goal rejected: vehicle is not armed"))
    s.enable()
    assert "vehicle is not armed" in s.approve(s.pending.hash[:12])
    assert s.pending is not None


def test_observe_only_never_flies():
    executor = FakeExecutor()
    s = flight_session(executor, autonomy="observe")      # submit_plan is denied, nothing pending
    assert s.pending is None
    plan = parse_plan(STEPS)
    s.pending = PendingPlan(plan, plan_hash(plan), time.monotonic())
    s.enable()
    assert s.approve(s.pending.hash[:12]).startswith("NOT STARTED (observe_only)")
    assert executor.started == []


def test_a_wrong_hash_never_reaches_the_executor():
    executor = FakeExecutor()
    s = flight_session(executor)
    s.enable()
    assert "does not match" in s.approve("0" * 12)
    assert executor.calls == []


def test_disable_stops_a_running_plan():
    executor = FakeExecutor()
    s = flight_session(executor)
    s.enable()
    s.approve(s.pending.hash[:12])
    assert "Stopped" in s.disable()
    assert "stop" in executor.calls
    assert not s.guard.armed


def test_disable_with_nothing_running_does_not_call_stop():
    executor = FakeExecutor()
    s = flight_session(executor)
    s.enable()
    assert s.disable() == "Agent disabled."
    assert executor.calls == []


def test_pause_resume_stop_and_status_go_to_the_executor():
    executor = FakeExecutor()
    s = flight_session(executor)
    s.enable()
    s.approve(s.pending.hash[:12])
    assert s.status().startswith("RUNNING")
    s.pause()
    assert s.status().startswith("PAUSED")
    s.resume()
    assert s.status().startswith("RUNNING")
    s.stop()
    assert s.status().startswith("ABORTED") and "cancelled" in s.status()
    assert executor.calls == ["execute", "pause", "resume", "stop"]


def test_operator_commands_without_an_executor_say_so():
    s = flight_session(None)
    assert "dry run" in s.pause()
    assert "dry run" in s.resume()
    assert "dry run" in s.stop()
    assert "dry run" in s.status()


def test_without_an_executor_approve_is_still_a_dry_run():
    s = flight_session(None)
    assert "Dry run" in s.approve(s.pending.hash[:12])
    assert s.pending is None


CONTINUE_STEPS = [{"skill": "goto", "args": {"x": 5, "y": 0, "altitude_m": 5}}, {"skill": "land"}]
IN_THE_AIR = FakeRobot(Telemetry(0.0, 0.0, 10.0, 87.0, True, "OFFBOARD"))


def test_a_plan_submitted_while_airborne_compiles_without_takeoff():
    s = flight_session(FakeExecutor(), steps=CONTINUE_STEPS, robot=IN_THE_AIR)
    assert s.pending is not None and s.pending.airborne


def test_the_same_plan_on_the_ground_is_rejected():
    s = flight_session(FakeExecutor(), steps=CONTINUE_STEPS)
    assert s.pending is None


class NoTelemetryRobot(FakeRobot):
    def get_telemetry(self):
        raise RobotUnavailable("nothing received yet on /fmu/out/vehicle_local_position_v1")


def test_unknown_telemetry_is_planned_as_on_the_ground_and_the_card_says_so():
    s = flight_session(FakeExecutor(), robot=NoTelemetryRobot())
    assert s.pending is not None and s.pending.airborne is None
    assert "Start state unknown" in render_card(s.pending.plan, airborne=s.pending.airborne)
