from dataclasses import dataclass

from drone_agent_core.controllers import Vec3
from drone_agent_core.frames import world_to_body
from drone_agent_core.goal import accept_goal 
from drone_agent_core.limits import HOLD
from drone_agent_core.runner import Runner

OVERRIDING_SOURCES = {"AUTO", "NAV2", "TELEOP"}        # other sources of offboard commands


@dataclass(frozen=True)
class Control:
    source: str       # published by the mux
    armed: bool
    
    
@dataclass(frozen=True)
class CoreOutput:
    velocity: Vec3       # body frame, ready for the mux
    step: int
    state: str           # IDLE, RUNNING, PAUSED, DONE or ABORTED
    message: str | None = None
    
    @property
    def finished(self) -> bool:
        return self.state in ("DONE", "ABORTED")   
    
    
class ExecutorCore:
    def __init__(self):
        self._runner: Runner | None = None
        
    @property
    def busy(self) -> None:
        return self._runner is not None
    
    def start(self, plan_json: str, plan_hash: str, control: Control) -> str | None:
        """None if the goal is accepted, else the reason it is refused"""
        if self._runner is not None:
            return "goal rejected: a plan is already running"
        if not control.armed:
            return "goal rejected: vehicle is not armed"
        runner, error = accept_goal(plan_json, plan_hash)
        if error is not None:
            return error
        self._runner = runner
        return None
    
    def pause(self, now: float) -> None:
        if self._runner is not None:
            self._runner.pause(now)
            
    def resume(self, now: float) -> None:
        if self._runner is not None:
            self._runner.resume(now)
            
    def cancel(self) -> None:
        if self._runner is not None:
            self._runner.abort("cancelled by the operator")
            
    def tick(self, position: Vec3 | None, yaw: float, control: Control, now: float) -> CoreOutput:
        runner = self._runner
        if runner is None:
            return CoreOutput(HOLD, 0, "IDLE")
        if not control.armed:
            runner.abort("vehicle disarmed")
        elif control.source in OVERRIDING_SOURCES:
            runner.pause(now)
        if position is None:
            runner.abort("position unavailable")
            position = HOLD             # not used: an aborted runner ignores it
        result = runner.tick(position, now)
        output = CoreOutput(world_to_body(result.velocity, yaw), result.step, result.state.name, result.message)
        if output.finished:
            self._runner = None
        return output
            