"""Web API for the dashboard's Mission console.

POST /api/agent/chat  {"messages": [{"role": "user", "content": "..."}]}  ->  {"content": "..."}

It drives the same Session and the same operator commands as the terminal chat
(`python3 -m drone_agent_core.chat`). Every request needs the access token; the
server binds to this computer only by default, and the dashboard's own server
forwards to it. The model still cannot approve or fly anything: /approve,
/enable and the rest are commands typed by the operator.
"""
import argparse
import hmac
import json
import os
import secrets
import sys
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlsplit

from drone_agent_core.chat import build_session, converse, make_executor, make_robot

CHAT_PATH = "/api/agent/chat"
HEALTH_PATH = "/api/agent/health"
MAX_BODY_BYTES = 64 * 1024
MAX_TEXT_CHARS = 4000
# Safe to run while the model is thinking: they only touch the executor or flip the interlock off.
ALWAYS_AVAILABLE = ("/stop", "/pause", "/resume", "/disable", "/status")
BUSY_TEXT = ("Snappy is busy with another request. "
             "/stop, /pause, /resume, /disable and /status still work.")


class WebApp:
    """The request logic, without sockets: easy to test."""

    def __init__(self, session, token: str):
        if not token:
            raise ValueError("an access token is required")
        self._session = session
        self._token = token
        self._busy = threading.Lock()      # one model turn or state change at a time

    def handle(self, method: str, path: str, headers: dict, body: bytes) -> tuple[int, dict]:
        """(HTTP status, JSON body). `headers` has lower-case names."""
        if path not in (CHAT_PATH, HEALTH_PATH):
            return 404, {"error": "not found"}
        if not self._same_origin(headers):
            return 403, {"error": "cross-origin requests are not allowed"}
        if not self._authorized(headers.get("authorization", "")):
            return 401, {"error": "missing or wrong access token"}
        if path == HEALTH_PATH:
            return (200, {"ok": True}) if method == "GET" else (405, {"error": "use GET"})
        if method != "POST":
            return 405, {"error": "use POST"}
        if not headers.get("content-type", "").lower().startswith("application/json"):
            return 415, {"error": "send application/json"}
        if len(body) > MAX_BODY_BYTES:
            return 413, {"error": "request too large"}
        text, error = self._last_user_text(body)
        if error is not None:
            return 400, {"error": error}
        try:
            return 200, {"content": self._run(text)}
        except Exception as e:                       # never leak internals to the browser
            print(f"web: request failed: {e!r}", file=sys.stderr)
            return 500, {"error": "Snappy hit an internal error; see the server log"}

    def _authorized(self, authorization: str) -> bool:
        scheme, _, value = authorization.partition(" ")
        if scheme.lower() != "bearer":
            return False
        return hmac.compare_digest(value.strip().encode(), self._token.encode())

    @staticmethod
    def _same_origin(headers: dict) -> bool:
        origin = headers.get("origin")
        if not origin:
            return True                              # not a browser, or stripped by the dashboard server
        return urlsplit(origin).netloc == headers.get("host", "")

    @staticmethod
    def _last_user_text(body: bytes) -> tuple[str | None, str | None]:
        """Only the newest user message is used: Snappy keeps its own history."""
        try:
            data = json.loads(body)
        except ValueError:
            return None, "body must be JSON"
        messages = data.get("messages") if isinstance(data, dict) else None
        if not isinstance(messages, list) or not messages:
            return None, "messages must be a non-empty list"
        last = messages[-1]
        if not isinstance(last, dict) or last.get("role") != "user" or not isinstance(last.get("content"), str):
            return None, "the last message must be a user message with text"
        text = last["content"].strip()
        if not text or len(text) > MAX_TEXT_CHARS:
            return None, f"the message must have 1 to {MAX_TEXT_CHARS} characters"
        return text, None

    def _run(self, text: str) -> str:
        word = text.split(None, 1)[0]
        if text.startswith("/") and word in ALWAYS_AVAILABLE:
            return converse(self._session, text, label="")
        if not self._busy.acquire(blocking=False):
            return BUSY_TEXT
        try:
            return converse(self._session, text, label="")
        finally:
            self._busy.release()


def make_server(app: WebApp, host: str = "127.0.0.1", port: int = 8765) -> ThreadingHTTPServer:
    class Handler(BaseHTTPRequestHandler):
        def _serve(self):
            length = self.headers.get("Content-Length")
            body = b""
            if self.command == "POST":
                if length is None or not length.isdigit():
                    return self._send(411, {"error": "Content-Length is required"})
                if int(length) > MAX_BODY_BYTES:
                    return self._send(413, {"error": "request too large"})
                body = self.rfile.read(int(length))
            headers = {k.lower(): v for k, v in self.headers.items()}
            status, payload = app.handle(self.command, urlsplit(self.path).path, headers, body)
            self._send(status, payload)

        def _send(self, status: int, payload: dict):
            data = json.dumps(payload).encode()
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(data)

        do_GET = do_POST = do_PUT = do_DELETE = _serve

        def log_message(self, format, *args):        # keep the terminal for what matters
            pass

    return ThreadingHTTPServer((host, port), Handler)


def main():
    parser = argparse.ArgumentParser(description="Web API for the dashboard's Snappy console")
    parser.add_argument("--robot", choices=["fake", "ros"], default="fake")
    parser.add_argument("--executor", choices=["none", "fake", "ros"], default="none")
    parser.add_argument("--host", default="127.0.0.1", help="keep the default; the dashboard server forwards here")
    parser.add_argument("--port", type=int, default=8765)
    args = parser.parse_args()

    token = os.environ.get("SNAPPY_WEB_TOKEN") or secrets.token_urlsafe(16)
    robot = make_robot(args.robot)
    executor = make_executor(args.executor)
    server = make_server(WebApp(build_session(robot, executor), token), args.host, args.port)
    print(f"Snappy web API on http://{args.host}:{args.port}  (robot: {args.robot}, executor: {args.executor})")
    print(f"Open the dashboard once with  #snappy={token}  added to its address, for example:")
    print(f"  http://localhost:4173/#snappy={token}")
    print("Flights need /enable and an approved plan (/approve <hash>) typed in the Mission console. Ctrl+C stops.")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        for closable in (executor, robot):           # executor first: robot.close() shuts rclpy down
            if hasattr(closable, "close"):
                closable.close()


if __name__ == "__main__":
    main()
