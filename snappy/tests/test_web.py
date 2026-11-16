import json
import threading
import urllib.error
import urllib.request

from drone_agent_core.guard import Guard
from drone_agent_core.plans import SUBMIT_PLAN
from drone_agent_core.profile import Policy
from drone_agent_core.robot import FakeExecutor, FakeRobot
from drone_agent_core.session import ModelReply, Session, ToolCall
from drone_agent_core.tools import OBSERVE_TOOLS, build_registry
from drone_agent_core.web import BUSY_TEXT, CHAT_PATH, HEALTH_PATH, MAX_BODY_BYTES, WebApp, make_server

TOKEN = "secret-token"
STEPS = [{"skill": "takeoff", "args": {"altitude_m": 5}}, {"skill": "land"}]


class ScriptedModel:
    def __init__(self, *replies):
        self._replies = list(replies)

    def reply(self, messages):
        return self._replies.pop(0)


class CrashingModel:
    def reply(self, messages):
        raise RuntimeError("secret internal detail")


def make_app(*replies, model=None, executor=None):
    executor = executor or FakeExecutor()
    session = Session(model or ScriptedModel(*replies), FakeRobot(), Guard(Policy(), ("/fmu/in/*",)),
                      build_registry(OBSERVE_TOOLS + (SUBMIT_PLAN,)), executor=executor)
    return WebApp(session, TOKEN), executor


def headers(**changes):
    base = {"authorization": f"Bearer {TOKEN}", "content-type": "application/json", "host": "localhost:4173"}
    return {**base, **changes}


def chat(app, text, **changes):
    body = json.dumps({"messages": [{"role": "user", "content": text}]}).encode()
    return app.handle("POST", CHAT_PATH, headers(**changes), body)


def test_a_request_needs_the_right_token():
    app, _ = make_app(ModelReply("Hi"))
    assert chat(app, "hello", authorization="")[0] == 401
    assert chat(app, "hello", authorization="Bearer wrong")[0] == 401
    assert chat(app, "hello", authorization=f"Token {TOKEN}")[0] == 401
    assert chat(app, "hello") == (200, {"content": "Hi\n [no tools used]"})


def test_only_the_newest_user_message_is_used():
    app, _ = make_app(ModelReply("Hi"))
    body = json.dumps({"messages": [{"role": "user", "content": "old"},
                                    {"role": "assistant", "content": "older reply"},
                                    {"role": "user", "content": "hello"}]}).encode()
    status, payload = app.handle("POST", CHAT_PATH, headers(), body)
    assert status == 200
    assert [m["content"] for m in app._session.messages if m["role"] == "user"] == ["hello"]


def test_bad_requests_are_refused():
    app, _ = make_app()
    ok = json.dumps({"messages": [{"role": "user", "content": "x"}]}).encode()
    assert app.handle("GET", CHAT_PATH, headers(), b"")[0] == 405
    assert app.handle("POST", "/api/other", headers(), ok)[0] == 404
    assert app.handle("POST", CHAT_PATH, headers(**{"content-type": "text/plain"}), ok)[0] == 415
    assert app.handle("POST", CHAT_PATH, headers(), b"not json")[0] == 400
    assert app.handle("POST", CHAT_PATH, headers(), b'{"messages": []}')[0] == 400
    assistant_last = json.dumps({"messages": [{"role": "assistant", "content": "x"}]}).encode()
    assert app.handle("POST", CHAT_PATH, headers(), assistant_last)[0] == 400
    assert chat(app, "   ")[0] == 400
    assert chat(app, "x" * 4001)[0] == 400
    assert app.handle("POST", CHAT_PATH, headers(), b"x" * (MAX_BODY_BYTES + 1))[0] == 413


def test_a_foreign_origin_is_refused_before_anything_else():
    app, _ = make_app(ModelReply("Hi"))
    assert chat(app, "hello", origin="http://evil.example")[0] == 403
    assert chat(app, "hello", origin="http://localhost:4173")[0] == 200


def test_health_needs_the_token_too():
    app, _ = make_app()
    assert app.handle("GET", HEALTH_PATH, headers(authorization=""), b"")[0] == 401
    assert app.handle("GET", HEALTH_PATH, headers(), b"") == (200, {"ok": True})


def test_commands_work_and_unknown_ones_never_reach_the_model():
    app, executor = make_app()
    assert chat(app, "/enable") == (200, {"content": "Agent enabled: approved plans may fly."})
    assert "Unknown command /stpo" in chat(app, "/stpo")[1]["content"]
    assert executor.calls == []


def test_a_plan_is_drafted_and_then_approved_through_the_web_api():
    app, executor = make_app(ModelReply(tool_calls=(ToolCall("submit_plan", {"steps": STEPS}),)),
                             ModelReply("Drafted."))
    content = chat(app, "take off to 5 m and land")[1]["content"]
    assert "Drafted." in content and "PLAN " in content and "Starts on the ground." in content
    prefix = app._session.pending.hash[:12]
    assert f"/approve {prefix}" in content
    assert chat(app, "/enable")[0] == 200
    assert chat(app, f"/approve {prefix}")[1]["content"].startswith("APPROVED")
    assert len(executor.started) == 1


def test_while_busy_emergency_commands_still_work_but_nothing_else_does():
    app, executor = make_app(ModelReply("never used"))
    chat(app, "/enable")
    app._busy.acquire()                               # a model turn is "running"
    try:
        assert chat(app, "hello")[1]["content"] == BUSY_TEXT
        assert chat(app, "/approve abcdefgh")[1]["content"] == BUSY_TEXT
        assert chat(app, "/stop")[1]["content"] == "Stopped: holding position."
        assert "stop" in executor.calls
        assert chat(app, "/status")[0] == 200
    finally:
        app._busy.release()


def test_internal_errors_do_not_leak_details():
    app, _ = make_app(model=CrashingModel())
    status, payload = chat(app, "hello")
    assert status == 500 and "secret internal detail" not in json.dumps(payload)


def test_a_real_http_round_trip():
    app, _ = make_app(ModelReply("Hi"))
    server = make_server(app, "127.0.0.1", 0)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    try:
        url = f"http://127.0.0.1:{server.server_address[1]}{CHAT_PATH}"
        body = json.dumps({"messages": [{"role": "user", "content": "hello"}]}).encode()
        request = urllib.request.Request(url, body, {"Authorization": f"Bearer {TOKEN}",
                                                     "Content-Type": "application/json"})
        with urllib.request.urlopen(request) as response:
            assert json.load(response)["content"].startswith("Hi")
        bad = urllib.request.Request(url, body, {"Content-Type": "application/json"})
        try:
            urllib.request.urlopen(bad)
            assert False, "expected 401"
        except urllib.error.HTTPError as e:
            assert e.code == 401
    finally:
        server.shutdown()
        server.server_close()
