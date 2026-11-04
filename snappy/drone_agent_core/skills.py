from drone_agent_core.controllers import GotoConfig, Vec3, goto_velocity
from drone_agent_core.limits import HOLD

def hold_tick(position: Vec3, args: dict, elapsed_s: float, cfg: GotoConfig = GotoConfig()) -> tuple[Vec3, bool]:
    return HOLD, elapsed_s >= args["seconds"]

def takeoff_tick(position: Vec3, args: dict, elapsed_s: float, cfg: GotoConfig = GotoConfig()) -> tuple[Vec3, bool]:
    x, y, _ = position
    return goto_velocity(position, (x, y, args["altitude_m"]), cfg)

def goto_tick(position: Vec3, args: dict, elapsed_s: float, cfg: GotoConfig = GotoConfig()) -> tuple[Vec3, bool]:
    return goto_velocity(position, (args["x"], args["y"], args["altitude_m"]), cfg)


SKILL_TICKS = {"takeoff": takeoff_tick, "goto": goto_tick, "hold": hold_tick}