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

def describe_step(step: Step) -> str:
    a = _normalized_args(step)
    if step.skill == "takeoff":
        return f"takeoff to {a['altitude_m']:g} m"
    if step.skill == "goto":
        return f"go to east {a['x']:g} m, north {a['y']:g} m at {a['altitude_m']:g} m"
    if step.skill == "hold":
        return f"hold for {a['seconds']:g} s"
    if step.skill == "land":
        return "land"
    raise ValueError(f"no description for skill {step.skill}")

def render_card(plan: Plan, limits: PlanLimits = PlanLimits()) -> str:
    errors = compile_plan(plan, limits)
    if errors:
        raise ValueError("cannot render a plan that does not compile:" + "; ".join(errors))
    args = [_normalized_args(s) for s in plan.steps]
    highest = max((a["altitude_m"] for a in args if "altitude_m" in a), default=0.0)
    farthest = max((math.hypot(a["x"], a["y"]) for a in args if "x" in a), default=0.0)
    held = sum(a["seconds"] for a in args if "seconds" in a)
    lines = [f"PLAN {plan_hash(plan)[:12]}  ({len(plan.steps)}  steps)"]
    for i, step in enumerate(plan.steps, 1):
        lines.append(f"  {i}. {describe_step(step)}")
    lines.append(f"  highest {highest:g} m (limit {limits.max_altitude_m:g}), "
                 f"farthest {farthest:g} m from start (limit {limits.geofence_radius_m:g}), "
                 f"holding {held:g} s")
    lines.append("  Nothing runs: dry run only.")
    return "\n".join(lines)

def parse_plan(raw) -> Plan:
    """Shape check only: a list of {skill, args}. Meaning is compile_plan's job"""
    if not isinstance(raw, list):
        raise ValueError("steps must be a list")
    steps = []
    for i, item in enumerate(raw, 1):
        if not isinstance(item, dict) or "skill" not in item or set(item) - {"skill", "args"}:
            raise ValueError(f"step {i} must be an object with skill and optionally args")
        args = item.get("args", {})
        if not isinstance(item["skill"], str) or not isinstance(args, dict):
            raise ValueError(f"step {i}: skill must be a string and args an object")
        steps.append(Step(item["skill"], dict(args)))
    return Plan(tuple(steps))

def submit_plan(raw_steps, limits: PlanLimits = PlanLimits()) -> str:
    """What the model is told after proposing a plan: errors to fix, or the card"""
    try:
        plan = parse_plan(raw_steps)
    except ValueError as e:
        return f"PLAN REJECTED: {e}"
    errors = compile_plan(plan, limits)
    if errors:
        return f"PLAN REJECTED:\n" + "\n".join(f"  {e}" for e in errors)
    return render_card(plan, limits)