import math
import hashlib
import json

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


@dataclass
class PlanLimits:
    max_steps: int = 12
    max_altitude_m: float = 30.0
    geofence_radius_m: float = 50.0
    max_hold_s: float = 120.0


MISSION_SKILLS = (
    ToolDef("takeoff", "Climb to a height above the ground", Risk.MOTION, None, {"altitude_m": float}),
    ToolDef("goto", "Fly to a point, east/north in meters", Risk.MOTION, None, {"x": float, "y": float, "altitude_m": float}),
    ToolDef("hold", "Hover in place for few seconds", Risk.MOTION, None, {"seconds": float}),
    ToolDef("land", "Land at the current position", Risk.MOTION),
)

SKILLS = build_registry(MISSION_SKILLS)

def check_step(step: Step):
    return validate(SKILLS.get(step.skill), step.args)

def check_limits(step: Step, limits: PlanLimits) -> str | None:
    a = step.args
    if step.skill in ("takeoff", "goto") and not 0 < a["altitude_m"] <= limits.max_altitude_m:
        return f"altitude must be between 0 and at most {limits.max_altitude_m}"
    if step.skill == "goto" and not math.hypot(a["x"], a["y"]) <= limits.geofence_radius_m:
        return f"target is outside the {limits.geofence_radius_m} m geofence"
    if step.skill == "hold" and not 0 < a["seconds"] <= limits.max_hold_s:
        return f"seconds must be above 0 and at most {limits.max_hold_s}"
    return None

def compile_plan(plan: Plan, limits: PlanLimits = PlanLimits(), airborne: bool = False) -> list[str]:
    """Returns a list of errors that the model can use to improve the plan"""
    if not plan.steps:
        return ["plan has no steps"]
    errors = []
    if len(plan.steps) > limits.max_steps:
        errors.append(f"plan has {len(plan.steps)} steps, at most {limits.max_steps} allowed")
    for i, step in enumerate(plan.steps, 1):
        problem = check_step(step) or check_limits(step, limits)
        if problem:
            errors.append(f"step {i} ({step.skill}): {problem}")
        if step.skill == "takeoff":
            if airborne:
                errors.append(f"step {i} (takeoff): already airborne")
            airborne = True
        elif step.skill == "land":
            if not airborne:
                errors.append(f"step {i} (land): not airborne")
            airborne=False
        elif step.skill in SKILLS and not airborne:
            errors.append(f"step {i} ({step.skill}): takeoff first")
    if airborne:
        errors.append("plan must end with land")
    return errors

def _normalized_args(step: Step) -> dict:
    params = SKILLS[step.skill].params
    return {k: float(v) if params[k] is float else v for k, v in step.args.items()}

def canonical_json(plan: Plan) -> str:
    """Stable text form of a plan that compile_plan accepted: same plan, same bytes"""
    data = {"steps": [{"skill":s.skill, "args": _normalized_args(s)} for s in plan.steps]}
    return json.dumps(data, sort_keys=True, separators=(",", ":"), allow_nan=False)

def plan_hash(plan: Plan) -> str:
    return hashlib.sha256(canonical_json(plan).encode("utf-8")).hexdigest()
