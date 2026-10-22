from dataclasses import dataclass, field

from drone_agent_core.guard import Risk, ToolSpec


GRAPH_KINDS = ("topics", "nodes", "services")

@dataclass(frozen=True)
class ToolDef:
    name: str
    description: str
    risk: Risk
    ros_name: str | None = None
    params: dict[str, type] = field(default_factory=dict)

    def spec(self) -> ToolSpec:
        return ToolSpec(self.name, self.risk, self.ros_name)

def validate(tool: ToolDef | None, args: dict) -> str | None:
    """Returns None if the call is well formed, else a readable error"""
    if tool is None:
        return "unknown tool"
    missing = [p for p in tool.params if p not in args]
    if missing:
        return f"missing argument(s): {', '.join(sorted(missing))}"
    unknown = [a for a in args if a not in tool.params]
    if unknown:
        return f"unexpected argument(s): {', '.join(sorted(unknown))}"
    for name, expected in tool.params.items():
        value = args[name]
        if isinstance(value, bool) and expected is not bool:
            return f"argument {name} must be {expected.__name__}"
        if not isinstance(value, expected):
            return f"argument {name} must be {expected.__name__}"
    if tool.name == "list_ros_graph" and args["kind"] not in GRAPH_KINDS:
        return f"kind must be one of: {','.join(GRAPH_KINDS)}"
    return None


OBSERVE_TOOLS = (
    ToolDef("get_telemetry", "Position, attitude and battery of the drone.",
            Risk.OBSERVE, "/fmu/out/vehicle_local_position"),
    ToolDef("get_detections", "Latest YOLO detections from the camera.",
            Risk.OBSERVE, "/drone_agent/detections"),
    ToolDef("list_ros_graph", f"List ROS names. kind is one of: {', '.join(GRAPH_KINDS)}.",
            Risk.OBSERVE, None, {"kind": str}),
)

def build_registry(tools = OBSERVE_TOOLS) -> dict[str, ToolDef]:
    return {t.name: t for t in tools}