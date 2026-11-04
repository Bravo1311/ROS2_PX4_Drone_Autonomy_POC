import json

from drone_agent_core.plans import PlanLimits, compile_plan, parse_plan, plan_hash
from drone_agent_core.runner import Runner


def accept_goal(plan_json: str, expected_hash: str, airborne: bool = False,
                limits: PlanLimits = PlanLimits()) -> tuple[Runner | None, str | None]:
    """A runner if the goal is the plan the operator approved, else (None, reason)"""
    try:
        data = json.loads(plan_json)
    except ValueError as e:
        return None, f"goal rejected: not valid JSON ({e})"
    if not isinstance(data, dict) or "steps" not in data:
        return None, "goal rejected: expected an object with steps"
    try:
        plan = parse_plan(data["steps"])
    except ValueError as e:
        return None, f"goal rejected: {e}"
    errors = compile_plan(plan, limits, airborne)
    if errors:
        return None, "goal rejected: " + "; ".join(errors)
    if plan_hash(plan) != expected_hash:
        return None, "goal rejected: hash does not match the plan"
    return Runner(plan), None