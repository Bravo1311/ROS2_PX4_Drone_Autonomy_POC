import math
from dataclasses import dataclass

from drone_agent_core.controllers import Vec3

HOLD: Vec3 = (0.0, 0.0, 0.0)

@dataclass(frozen=True)
class ExecutorLimits:
    max_speed_mps: float = 2.0
    max_altitude_m: float = 35.0
    geofence_radius_m: float = 55.0
    margin_m: float = 0.5

    def __post_init__(self):
        if min(self.max_speed_mps, self.max_altitude_m, self.geofence_radius_m) <= 0 or self.margin_m < 0:
            raise ValueError("limits must be positive and margin_m must not be negative")

def limit_velocity(position: Vec3, velocity: Vec3, limits: ExecutorLimits = ExecutorLimits()) -> tuple[Vec3, str | None]:
    """The velocity that may be sent, and a reason if the plan must end (then the velocity is a hold)."""
    if not all(math.isfinite(c) for c in (*position, *velocity)):
        return HOLD, "position or velocity is not a finite number"
    x, y, altitude = position
    if altitude > limits.max_altitude_m + limits.margin_m:
        return HOLD, f"altitude {altitude:.1f} m is above the {limits.max_altitude_m:g} m limit"
    distance = math.hypot(x, y)
    if distance > limits.geofence_radius_m + limits.margin_m:
        return HOLD, f"{distance:.1f} m from the start is outside the {limits.geofence_radius_m:g} m geofence"
    speed = math.hypot(*velocity)
    if speed > limits.max_speed_mps:
        scale = limits.max_speed_mps / speed
        velocity = tuple(c * scale for c in velocity)
    return velocity, None   