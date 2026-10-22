from dataclasses import dataclass
from typing import Protocol

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