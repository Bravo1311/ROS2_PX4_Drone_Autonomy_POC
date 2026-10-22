from dataclasses import dataclass, field

from drone_agent_core.guard import Risk
from drone_agent_core.tools import ToolDef, build_registry, validate


@dataclass(frozen=True)
class Step:
    skill: str
    args: dict = field(default_factory=dict)

@dataclass
class Plan:
    steps: tuple[Step, ...]

MISSION_SKILLS = (
    ToolDef("takeoff", "Climb to a height above the ground", Risk.MOTION, None, {"altitude_m": float}),
    ToolDef("goto", "Fly to a point, east/north in meters", Risk.MOTION, None, {"x": float, "y": float, "altitude_m": float}),
    ToolDef("hold", "Hover in place for few seconds", Risk.MOTION, None, {"seconds": float}),
    ToolDef("land", "Land at the current position", Risk.MOTION),
)

SKILLS = build_registry(MISSION_SKILLS)

def check_step(step: Step):
    return validate(SKILLS.get(step.skill), step.args)
