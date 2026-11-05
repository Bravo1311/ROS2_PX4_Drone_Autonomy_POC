from drone_agent_core.chat import tool_status
from drone_agent_core.session import wrap_result


def test_data_is_ok():
    assert tool_status(wrap_result("get_telemetry", "{}")) == "ok"


def test_denials_and_unavailable_data_show_their_start():
    assert tool_status("DENIED (budget): too many calls").startswith("DENIED (budget)")
    assert tool_status("UNAVAILABLE: no data").startswith("UNAVAILABLE")
    assert tool_status("RATE LIMITED: try again in 1.0s").startswith("RATE LIMITED")


def test_a_rejected_plan_is_flagged_even_though_it_is_wrapped():
    content = wrap_result("submit_plan", "PLAN REJECTED: steps must be a list")
    assert tool_status(content) == "plan rejected"

# --- operator commands ------------------------------------------------------

from drone_agent_core.chat import HELP, make_executor, run_command
from drone_agent_core.robot import FakeExecutor
from tests.test_session_execution import flight_session


def test_an_unknown_slash_command_is_never_sent_to_the_model():
    s = flight_session(FakeExecutor())
    reply = run_command(s, "/stpo")
    assert reply.startswith("Unknown command /stpo") and HELP in reply
    assert s.executor.calls == []


def test_every_operator_command_is_routed():
    executor = FakeExecutor()
    s = flight_session(executor)
    assert run_command(s, "/enable") == "Agent enabled: approved plans may fly."
    assert run_command(s, f"/approve {s.pending.hash[:12]}").startswith("APPROVED")
    run_command(s, "/pause")
    run_command(s, "/resume")
    run_command(s, "/stop")
    assert executor.calls == ["execute", "pause", "resume", "stop"]
    assert run_command(s, "/status").startswith("ABORTED")
    assert run_command(s, "/reject") == "Nothing to reject."
    assert run_command(s, "/disable") == "Agent disabled."


def test_executor_kinds():
    assert make_executor("none") is None
    assert isinstance(make_executor("fake"), FakeExecutor)
