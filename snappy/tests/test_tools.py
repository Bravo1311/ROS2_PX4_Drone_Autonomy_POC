import pytest

from drone_agent_core.guard import Risk
from drone_agent_core.tools import ToolDef, build_registry, validate

MOVE = ToolDef("go_to", "Fly to a point.", Risk.MOTION, None, {"x": float, "y": int})


def test_valid_call_passes():
    assert validate(MOVE, {"x": 1.5, "y": 2}) is None


def test_unknown_tool():
    assert validate(None, {}) == "unknown tool"


@pytest.mark.parametrize("args, fragment", [
    ({"x": 1.5}, "missing"),
    ({"x": 1.5, "y": 2, "z": 3}, "unexpected"),
    ({"x": "a", "y": 2}, "x must be float"),
    ({"x": 1.5, "y": True}, "y must be int"),
])
def test_bad_calls_get_readable_errors(args, fragment):
    assert fragment in validate(MOVE, args)


def test_registry_looks_up_by_name_and_misses_cleanly():
    registry = build_registry()
    assert registry["get_telemetry"].risk == Risk.OBSERVE
    assert registry.get("fly_away") is None


def test_spec_carries_name_risk_and_ros_name():
    spec = build_registry()["get_telemetry"].spec()
    assert (spec.name, spec.risk) == ("get_telemetry", Risk.OBSERVE)
    assert spec.ros_name == "/fmu/out/vehicle_local_position"

def test_list_ros_graph_rejects_unknown_kind():
    tool = build_registry()["list_ros_graph"]
    assert validate(tool, {"kind": "topic"}) is not None
    assert validate(tool, {"kind": "topics"}) is None