from drone_agent_core.guard import Guard
from drone_agent_core.ollama_model import OllamaModel
from drone_agent_core.profile import Policy
from drone_agent_core.robot import FakeRobot
from drone_agent_core.session import Session
from drone_agent_core.tools import build_registry, OBSERVE_TOOLS
from drone_agent_core.plans import SUBMIT_PLAN, render_card

import argparse


MODEL_NAME = "qwen2.5:3b"
HARD_DENY = ("/fmu/in/*", "*cmd_vel*")
PROBLEM_PREFIXES = ("DENIED", "UNAVAILABLE", "RATE LIMITED")

def make_robot(kind: str):
    if kind == "ros":
        # needs ROS; imported only when asked
        from drone_agent_core.ros_robot import RosRobot
        return RosRobot()
    return FakeRobot()

def build_session(robot) -> Session:
    registry = build_registry(OBSERVE_TOOLS + (SUBMIT_PLAN, ))
    model = OllamaModel(model=MODEL_NAME, tools=registry.values())
    policy = Policy()
    guard = Guard(policy, HARD_DENY)
    return Session(model, robot, guard, registry, approval_ttl_s=policy.approval_ttl_s)

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

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot", choices=["fake", "ros"], default="fake")
    args = parser.parse_args()

    robot = make_robot(args.robot)
    session = build_session(robot)
    print(f"Snappy - An AI assistant for talking to your robot: {args.robot}. Type /quit to leave.")
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
            if text.startswith("/approve"):
                print(session.approve(text.removeprefix("/approve").strip()))
                continue
            if text == "/reject":
                print(session.reject())
                continue
            before = session.pending
            start = len(session.messages)
            answer = session.handle_user_message(text)
            print("Snappy> ", answer)
            print(tool_log(session, start))
            if session.pending is not None and session.pending is not before:
                print(render_card(session.pending.plan))
                print(f"   /approve {session.pending.hash[:12]}   or   /reject ")
    finally:
        if hasattr(robot, "close"):
            robot.close()

if __name__=="__main__":
    main()