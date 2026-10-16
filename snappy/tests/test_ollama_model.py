import pytest

from drone_agent_core.guard import Guard, Risk
from drone_agent_core.ollama_model import (
    OllamaError, OllamaModel, parse_reply, to_ollama_messages, tool_schema,
)
from drone_agent_core.profile import Policy
from drone_agent_core.robot import FakeRobot
from drone_agent_core.session import ModelReply, Session, ToolCall
from drone_agent_core.tools import ToolDef, build_registry, validate

GRAPH = ToolDef("list_ros_graph", "List ROS names.", Risk.OBSERVE, None, {"kind": str})
PING = ToolDef("get_telemetry", "Telemetry.", Risk.OBSERVE)


def answer(content="", tool_calls=None):
    message = {"role": "assistant", "content": content}
    if tool_calls is not None:
        message["tool_calls"] = tool_calls
    return {"message": message, "done": True}


def fn(name, arguments):
    return {"function": {"name": name, "arguments": arguments}}


def test_tool_schema_with_a_parameter():
    assert tool_schema(GRAPH) == {
        "type": "function",
        "function": {
            "name": "list_ros_graph",
            "description": "List ROS names.",
            "parameters": {
                "type": "object",
                "properties": {"kind": {"type": "string"}},
                "required": ["kind"],
            },
        },
    }


def test_tool_schema_without_parameters():
    params = tool_schema(PING)["function"]["parameters"]
    assert params == {"type": "object", "properties": {}, "required": []}


def test_messages_are_converted_to_the_ollama_format():
    history = [
        {"role": "user", "content": "battery?"},
        {"role": "assistant", "content": "", "tool_calls": [{"name": "get_telemetry", "args": {}}]},
        {"role": "tool", "name": "get_telemetry", "content": "{}"},
        {"role": "assistant", "content": "87%", "tool_calls": []},
    ]
    out = to_ollama_messages(history, "SYS")
    assert out[0] == {"role": "system", "content": "SYS"}
    assert out[2]["tool_calls"] == [fn("get_telemetry", {})]
    assert out[3] == {"role": "tool", "tool_name": "get_telemetry", "content": "{}"}
    assert out[4] == {"role": "assistant", "content": "87%"}


def test_text_reply():
    assert parse_reply(answer("hello")) == ModelReply("hello")


def test_tool_call_with_dict_arguments():
    reply = parse_reply(answer(tool_calls=[fn("list_ros_graph", {"kind": "topics"})]))
    assert reply.tool_calls == (ToolCall("list_ros_graph", {"kind": "topics"}),)


def test_tool_call_with_json_string_arguments():
    reply = parse_reply(answer(tool_calls=[fn("list_ros_graph", '{"kind": "topics"}')]))
    assert reply.tool_calls == (ToolCall("list_ros_graph", {"kind": "topics"}),)


def test_unreadable_arguments_become_a_marker_that_validation_rejects():
    call = parse_reply(answer(tool_calls=[fn("list_ros_graph", "not json")])).tool_calls[0]
    assert call.args == {"_unparsed": "not json"}
    assert validate(GRAPH, call.args) is not None


def test_malformed_response_raises():
    with pytest.raises(OllamaError):
        parse_reply({"oops": 1})


def test_reply_sends_the_expected_request():
    sent = {}

    def fake_post(url, payload, timeout):
        sent.update(url=url, payload=payload, timeout=timeout)
        return answer("hi")

    model = OllamaModel("qwen2.5:3b", [GRAPH, PING], post=fake_post)
    assert model.reply([{"role": "user", "content": "hello"}]).text == "hi"

    payload = sent["payload"]
    assert sent["url"] == "http://localhost:11434/api/chat"
    assert payload["model"] == "qwen2.5:3b" and payload["stream"] is False
    assert [t["function"]["name"] for t in payload["tools"]] == ["list_ros_graph", "get_telemetry"]
    assert payload["messages"][0]["role"] == "system"
    assert payload["messages"][-1] == {"role": "user", "content": "hello"}


def test_session_runs_end_to_end_with_the_ollama_model():
    responses = iter([answer(tool_calls=[fn("get_telemetry", {})]), answer("Battery is 87%.")])
    seen = []

    def fake_post(url, payload, timeout):
        seen.append(payload)
        return next(responses)

    registry = build_registry()
    model = OllamaModel("qwen2.5:3b", registry.values(), post=fake_post)
    robot = FakeRobot()
    session = Session(model, robot, Guard(Policy(), ("/fmu/in/*",)), registry)

    assert session.handle_user_message("battery?") == "Battery is 87%."
    assert robot.calls == ["get_telemetry"]
    assert any(m["role"] == "tool" and m["tool_name"] == "get_telemetry"
               for m in seen[1]["messages"])