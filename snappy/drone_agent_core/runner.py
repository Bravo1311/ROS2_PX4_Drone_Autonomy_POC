from dataclasses import dataclass
from enum import Enum

from drone_agent_core.controllers import GotoConfig, Vec3
from drone_agent_core.limits import ExecutorLimits, HOLD, limit_velocity
from drone_agent_core.plans import Plan
from drone_agent_core.skills import SKILL_TICKS

class RunState(Enum):
    RUNNING = "running"
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
        self.state = RunState.RUNNING
        self.message: str | None = None
        
    def tick(self, position: Vec3, now: float) -> TickResult:
        if self.state is not RunState.RUNNING:
            return TickResult(HOLD, self._index, self.state, self.message)
        step = self._plan.steps[self._index]
        if self._step_started_at is None:
            self._step_started_at = now
        velocity, done = SKILL_TICKS[step.skill](position, step.args, now - self._step_started_at, self._cfg)
        velocity, breach = limit_velocity(position, velocity, self._limits)
        if breach is not None:
            self.state = RunState.ABORTED
            self.message = breach
            return TickResult(HOLD, self._index, self.state, self.message)
        if done:
            self._index += 1
            self._step_started_at = None
            if self._index == len(self._plan.steps):
                self.state = RunState.DONE
        return TickResult(velocity, self._index, self.state, self.message)