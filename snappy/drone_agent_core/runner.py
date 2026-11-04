from dataclasses import dataclass
from enum import Enum

from drone_agent_core.controllers import GotoConfig, Vec3
from drone_agent_core.limits import ExecutorLimits, HOLD, limit_velocity
from drone_agent_core.plans import Plan
from drone_agent_core.skills import SKILL_TICKS, SKILL_EXPECTED_S

TIMEOUT_FACTOR = 3.0
TIMEOUT_SLACK_S = 10.0

class RunState(Enum):
    RUNNING = "running"
    PAUSED = "paused"
    DONE = "done"
    ABORTED = "aborted"
    
@dataclass(frozen=True)
class TickResult:
    velocity: Vec3
    step: int                     # index of the step that runs next; len(steps) when done
    state: RunState
    message: str | None = None    # the reason if aborted
    

class Runner:
    def __init__(self, plan: Plan, limits: ExecutorLimits = ExecutorLimits(), goto_cfg: GotoConfig = GotoConfig()):
        if not plan.steps:
            raise ValueError("plan has no steps")
        missing = sorted({s.skill for s in plan.steps} - set(SKILL_TICKS)) 
        if missing:
            raise ValueError(f"No implementation for skill(s): {', '.join(missing)}")
        self._plan = plan
        self._limits = limits
        self._cfg = goto_cfg
        self._index = 0
        self._step_started_at: float | None = None
        self._step_deadline: float | None = None
        self.state = RunState.RUNNING
        self.message: str | None = None
        self._paused_at: float | None = None
        
    def tick(self, position: Vec3, now: float) -> TickResult:
        if self.state is not RunState.RUNNING:
            return TickResult(HOLD, self._index, self.state, self.message)
        step = self._plan.steps[self._index]
        if self._step_started_at is None:
            self._step_started_at = now
            expected = SKILL_EXPECTED_S[step.skill](position, step.args, self._cfg)
            self._step_deadline = now + TIMEOUT_FACTOR * expected + TIMEOUT_SLACK_S
        velocity, done = SKILL_TICKS[step.skill](position, step.args, now - self._step_started_at, self._cfg)
        velocity, breach = limit_velocity(position, velocity, self._limits)
        if breach is not None:
            self.state = RunState.ABORTED
            self.message = breach
            return TickResult(HOLD, self._index, self.state, self.message)
        if not done and now > self._step_deadline:
            self.state = RunState.ABORTED
            self.message = f"step {self._index + 1} ({step.skill}) timed out"
            return TickResult(HOLD, self._index, self.state, self.message)
        if done:
            self._index += 1
            self._step_started_at = None
            self._step_deadline = None
            if self._index == len(self._plan.steps):
                self.state = RunState.DONE
        return TickResult(velocity, self._index, self.state, self.message)
    
    def pause(self, now: float) -> None:
        if self.state is RunState.RUNNING:
            self.state = RunState.PAUSED
            self._paused_at = now
            
    def resume(self, now: float) -> None:
        if self.state is not RunState.PAUSED:
            return
        pause_length = now - self._paused_at
        if self._step_started_at is not None:
            self._step_started_at += pause_length
            self._step_deadline += pause_length
        self._paused_at = None
        self.state = RunState.RUNNING