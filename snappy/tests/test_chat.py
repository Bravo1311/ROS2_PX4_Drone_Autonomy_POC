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