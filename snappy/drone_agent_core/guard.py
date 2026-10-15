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
        
        self._max_calls_per_turn = self._policy.max_calls_per_turn
        self._counter = 0
        
        self._repeat_counter_limit = self._policy.repeat_limit
        self._repeat_counter = 0
        self._last_tool_call = None

        self._invalid_count = 0

    def arm(self):
        self.armed = True

    def disarm(self):
        self.armed = False

    def begin_turn(self):
        self._counter = 0
        self._repeat_counter = 0
        self._last_tool_call = None
        self._invalid_count = 0

    def _is_repeat(self, tool_name: str, args: dict) -> bool:
        call = (tool_name, dict(args))
        if call == self._last_tool_call:
            self._repeat_counter += 1
        else:
            self._repeat_counter = 1
            self._last_tool_call = call
        return self._repeat_counter > self._repeat_counter_limit

    def record_invalid(self, reason: str) -> Decision:
        self._counter += 1
        self._invalid_count += 1
        if self._invalid_count > self._policy.max_invalid_per_turn:
            return Decision(
                Verdict.DENY, "invalid_cap",
                "Too many malformed tool calls this turn"
            )
        return Decision(Verdict.DENY, "invalid", reason)

    def decide(self, tool: ToolSpec, args: dict | None = None):
        # matches ros names against forbidden ones
        # hard deny
        self._counter += 1
        repeat_check_result = self._is_repeat(tool.name, args or {})

        if tool.ros_name is not None:
            for pattern in self._hard_deny:
                if fnmatch(tool.ros_name, pattern):
                    return Decision(
                        Verdict.DENY, "hard_deny",
                        f"{tool.ros_name} is never allowed for the agent"
                    )
        # budget
        if (self._counter > self._max_calls_per_turn):
            return Decision(
                Verdict.DENY, "budget",
                "The tool call exceeds maximum calls allowed per turn. Start afresh"
            )

        # repeat 
        if (repeat_check_result):
            return Decision(
                Verdict.DENY, "repeat",
                "The same tool call was repeated too many times in a row"
            )
        # Observe-only
        if self._policy.autonomy == "observe" and tool.risk != Risk.OBSERVE:
            return Decision(
                Verdict.DENY, "observe_only",
                "the agent is in observe-only mode"
            )
        # Disarmed
        if tool.risk == Risk.MOTION and not self.armed:
            return Decision(Verdict.DENY, "disarmed", "the drone is disarmed")
        # Supervised
        if tool.risk == Risk.MOTION:
            return Decision(
                Verdict.NEED_APPROVAL, "supervised", "the operator must approve"
            )
        return Decision(Verdict.ALLOW)