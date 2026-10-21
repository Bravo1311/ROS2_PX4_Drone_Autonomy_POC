from drone_agent_core.guard import Guard
from drone_agent_core.ollama_model import OllamaModel
from drone_agent_core.profile import Policy
from drone_agent_core.robot import FakeRobot
from drone_agent_core.session import Session
from drone_agent_core.tools import build_registry

MODEL_NAME = "qwen2.5:3b"
HARD_DENY = ("/fmu/in/*", "*cmd_vel*")


def build_session() -> Session:
    registry = build_registry()
    model = OllamaModel(model=MODEL_NAME, tools=registry.values())
    guard = Guard(Policy(), HARD_DENY)
    return Session(model, FakeRobot(), guard, registry)

def main():
    session = build_session()
    print("Snappy (simulated robot). Type /quit to leave.")
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
        print("Snappy> ", session.handle_user_message(text))

if __name__=="__main__":
    main()