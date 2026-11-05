from drone_agent_core.guard import Guard
from drone_agent_core.ollama_model import OllamaModel
from drone_agent_core.profile import Policy
from drone_agent_core.robot import FakeExecutor, FakeRobot
from drone_agent_core.session import Session
from drone_agent_core.tools import build_registry, OBSERVE_TOOLS
from drone_agent_core.plans import SUBMIT_PLAN, render_card

import argparse


MODEL_NAME = "qwen2.5:3b"
HARD_DENY = ("/fmu/in/*", "*cmd_vel*")
PROBLEM_PREFIXES = ("DENIED", "UNAVAILABLE", "RATE LIMITED")
HELP = "Commands: /approve <hash>, /reject, /enable, /disable, /pause, /resume, /stop, /status, /quit"

def make_robot(kind: str):
    if kind == "ros":
        # needs ROS; imported only when asked
        from drone_agent_core.ros_robot import RosRobot
        return RosRobot()
    return FakeRobot()

def make_executor(kind: str):
    """None keeps the chat a dry run: /approve then executes nothing."""
    if kind == "ros":
        # needs ROS and the executor node; imported only when asked
        from drone_agent_core.ros_executor import RosExecutor
        return RosExecutor()
    if kind == "fake":
        return FakeExecutor()
    return None

def build_session(robot, executor=None) -> Session:
    registry = build_registry(OBSERVE_TOOLS + (SUBMIT_PLAN, ))
    model = OllamaModel(model=MODEL_NAME, tools=registry.values())
    policy = Policy()
    guard = Guard(policy, HARD_DENY)
    return Session(model, robot, guard, registry, approval_ttl_s=policy.approval_ttl_s,
                   executor=executor)

def tool_status(content: str) -> str:
    """'ok' for data; the start of the message for denials, unavailable data, rate limits"""
    if content.startswith(PROBLEM_PREFIXES):
        return content[:100]
    if "PLAN REJECTED" in content:
        return "plan rejected"
    return 'ok'

def tool_log(session: Session, start: int) -> str:
    """The tool calls made since message number 'start', one per line"""
    lines = [f" [{m['name']}: {tool_status(m['content'])}]"
             for m in session.messages[start:] if m["role"] == "tool"]
    return "\n".join(lines) if lines else " [no tools used]"

def run_command(session: Session, text: str) -> str:
    """The reply to an operator command. Text starting with / never goes to the model."""
    word, _, rest = text.partition(" ")
    if word == "/approve":
        return session.approve(rest.strip())
    simple = {
        "/reject": session.reject, "/enable": session.enable, "/disable": session.disable,
        "/pause": session.pause, "/resume": session.resume, "/stop": session.stop,
        "/status": session.status,
    }
    if word in simple:
        return simple[word]()
    return f"Unknown command {word}. {HELP}"

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot", choices=["fake", "ros"], default="fake")
    parser.add_argument("--executor", choices=["none", "fake", "ros"], default="none")
    args = parser.parse_args()

    robot = make_robot(args.robot)
    executor = make_executor(args.executor)
    session = build_session(robot, executor)
    print(f"Snappy - An AI assistant for talking to your robot: {args.robot}, executor: {args.executor}.")
    print(HELP)
    try:
        while True:
            try:
                text = input("you> ").strip()
            except (EOFError, KeyboardInterrupt):
                print()
                break
            if not text:
                continue
            if text == "/quit":
                break
            if text.startswith("/"):
                print(run_command(session, text))
                continue
            before = session.pending
            start = len(session.messages)
            answer = session.handle_user_message(text)
            print("Snappy> ", answer)
            print(tool_log(session, start))
            if session.pending is not None and session.pending is not before:
                print(render_card(session.pending.plan, airborne=session.pending.airborne))
                print(f"   /approve {session.pending.hash[:12]}   or   /reject ")
    finally:
        for closable in (executor, robot):      # executor first: robot.close() shuts rclpy down
            if hasattr(closable, "close"):
                closable.close()

if __name__=="__main__":
    main()