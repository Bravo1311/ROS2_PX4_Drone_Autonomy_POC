from dataclasses import dataclass
from enum import Enum
from fnmatch import fnmatch

from drone_agent_core.profile import Policy

class Risk(Enum):
    OBSERVE = "observe"
    ANNOTATE = "annotate"
    MOTION = "motion"

class Verdict(Enum):
    ALLOW = "allow"
    NEED_APPROVAL = "need_approval"
    DENY = "deny"

@dataclass(frozen=True)
class ToolSpec:
    name: str 
    risk: Risk
    ros_name: str | None = None

@dataclass(frozen=True)
class Decision:
    verdict: Verdict
    rule: str = ""
    message: str = ""

class Guard:
    def __init__(self, policy: Policy, hard_deny=()):
        self._policy = policy
        self._hard_deny = tuple(hard_deny)
        self.armed = policy.start_armed

    def arm(self):
        self.armed = True

    def disarm(self):
        self.armed = False

    def decide(self, tool: ToolSpec):
        if tool.ros_name is not None:
            for pattern in self._hard_deny:
                if fnmatch(tool.ros_name, pattern):
                    return Decision(
                        Verdict.DENY, "hard_deny",
                        f"{tool.ros_name} is never allowed for the agent"
                    )
        if tool.risk == Risk.MOTION and not self.armed:
            return Decision(Verdict.DENY, "disarmed", "the drone is disarmed")
        if self._policy.autonomy == "observe" and tool.risk != Risk.OBSERVE:
            return Decision(
                Verdict.DENY, "observe_only",
                "the agent is in observe-only mode"
            )
        if tool.risk == Risk.MOTION:
            return Decision(
                Verdict.NEED_APPROVAL, "supervised", "the operator must approve"
            )
        return Decision(Verdict.ALLOW)