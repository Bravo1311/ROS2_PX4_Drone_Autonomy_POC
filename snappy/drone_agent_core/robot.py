from dataclasses import dataclass
from typing import Protocol

@dataclass(frozen=True)
class ExecutionStatus:
    state: str                  # IDLE, RUNNING, PAUSED, DONE or ABORTED
    step: int = 0               # the step that runs next
    message: str | None = None  # the reason, if it was aborted

@dataclass(frozen=True)
class Telemetry:
    x: float
    y: float
    altitude_m: float
    battery_pct: float | None
    armed: bool        #PX4's armed state
    mode: str

@dataclass(frozen=True)
class Detection:
    label: str
    confidence: float

class RobotPort(Protocol):
    def get_telemetry(self) -> Telemetry: ...
    def get_detections(self) -> list[Detection]: ...
    def list_ros_graph(self, kind: str) -> list[str]: ...

class ExecutorPort(Protocol):
    def execute(self, plan_json: str, plan_hash: str) -> None:
        """None if the flight started, else the reason it was refused"""
        ...
    def pause(self) -> str: ...
    def resume(self) -> str: ...
    def stop(self) -> str: ...
    def status(self) -> ExecutionStatus: ...

class RobotUnavailable(Exception):
    """The robot cannot provide data right now (nothing received yet, or too old)"""

class FakeRobot:
    """Scripted robot for tests and the offline chat."""

    def __init__(self, telemetry=None, detections=None, graph=None):
        self.telemetry = telemetry or Telemetry(0.0, 0.0, 0.0, 87.0, False, "HOLD")
        self.detections = list(detections or [])
        self.graph = dict(graph or {"topics": ["/fmu/out/vehicle_status"]})
        self.calls: list[str] = []

    def get_telemetry(self) -> Telemetry:
        self.calls.append("get_telemetry")
        return self.telemetry

    def get_detections(self) -> list[Detection]:
        self.calls.append("get_detections")
        return list(self.detections)

    def list_ros_graph(self, kind: str) -> list[str]:
        self.calls.append("list_ros_graph")
        return list(self.graph.get(kind, []))
    

class FakeExecutor:
    """Records what the session asks for; for tests and the offline chat."""

    def __init__(self, refuse_with: str | None = None):
        self.refuse_with = refuse_with
        self.started: list[tuple[str, str]] = []     # (plan_json, plan_hash) of accepted flights
        self.calls: list[str] = []
        self._status = ExecutionStatus("IDLE")

    def execute(self, plan_json: str, plan_hash: str) -> str | None:
        self.calls.append("execute")
        if self.refuse_with is not None:
            return self.refuse_with
        self.started.append((plan_json, plan_hash))
        self._status = ExecutionStatus("RUNNING")
        return None

    def pause(self) -> str:
        self.calls.append("pause")
        self._status = ExecutionStatus("PAUSED", self._status.step)
        return "Paused."

    def resume(self) -> str:
        self.calls.append("resume")
        self._status = ExecutionStatus("RUNNING", self._status.step)
        return "Resumed."

    def stop(self) -> str:
        self.calls.append("stop")
        self._status = ExecutionStatus("ABORTED", self._status.step, "cancelled by the operator")
        return "Stopped: holding position."

    def status(self) -> ExecutionStatus:
        return self._status