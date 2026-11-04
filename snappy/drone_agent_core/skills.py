import math

from drone_agent_core.controllers import GotoConfig, Vec3, goto_velocity
from drone_agent_core.limits import HOLD

LAND_SPEED_MPS = 0.5
TOUCHDOWN_ALTITUDE_M = 0.15

def hold_tick(position: Vec3, args: dict, elapsed_s: float, cfg: GotoConfig = GotoConfig()) -> tuple[Vec3, bool]:
    return HOLD, elapsed_s >= args["seconds"]

def takeoff_tick(position: Vec3, args: dict, elapsed_s: float, cfg: GotoConfig = GotoConfig()) -> tuple[Vec3, bool]:
    x, y, _ = position
    return goto_velocity(position, (x, y, args["altitude_m"]), cfg)

def land_tick(position: Vec3, args: dict, elapsed_s: float, cfg: GotoConfig = GotoConfig()) -> tuple[Vec3, bool]:
    _, _, altitude = position
    if altitude <= TOUCHDOWN_ALTITUDE_M:
        return HOLD, True
    return (0.0, 0.0, -LAND_SPEED_MPS), False

def goto_tick(position: Vec3, args: dict, elapsed_s: float, cfg: GotoConfig = GotoConfig()) -> tuple[Vec3, bool]:
    return goto_velocity(position, (args["x"], args["y"], args["altitude_m"]), cfg)



SKILL_TICKS = {"takeoff": takeoff_tick, "goto": goto_tick, "hold": hold_tick, "land": land_tick}


def takeoff_expected_s(position: Vec3, args: dict, cfg: GotoConfig) -> float:
    return abs(args["altitude_m"] - position[2]) / cfg.max_speed_mps


def goto_expected_s(position: Vec3, args: dict, cfg: GotoConfig) -> float:
    return math.dist(position, (args["x"], args["y"], args["altitude_m"])) / cfg.max_speed_mps


def hold_expected_s(position: Vec3, args: dict, cfg: GotoConfig) -> float:
    return float(args["seconds"])


def land_expected_s(position: Vec3, args: dict, cfg: GotoConfig) -> float:
    return max(0.0, position[2] - TOUCHDOWN_ALTITUDE_M) / LAND_SPEED_MPS


SKILL_EXPECTED_S = {
    "takeoff": takeoff_expected_s,
    "goto": goto_expected_s,
    "hold": hold_expected_s,
    "land": land_expected_s,
}