import math
from dataclasses import dataclass

Vec3 = tuple[float, float, float]

@dataclass(frozen=True)
class GotoConfig:
    max_speed_mps: float = 1.5
    slow_radius_m: float = 3.0
    arrive_radius_m: float = 0.2

    def __post_init__(self):
        if self.max_speed_mps <= 0 or not 0 <self.arrive_radius_m <self.slow_radius_m:
            raise ValueError("need max_speed_mps > 0 and 0 < arrive_radius_m < slow_radius_m")

def goto_velocity(position: Vec3, target: Vec3, cfg: GotoConfig = GotoConfig()) -> tuple[Vec3, bool]:
    """vel in the ENU frame + whether we have arrived"""
    ex, ey, ez = (t - p for t, p in zip(target, position))
    distance = math.dist(position, target)
    if not math.isfinite(distance):
        return (0.0, 0.0, 0.0), False          # bad input: hold, never guess
    if distance <= cfg.arrive_radius_m:
        return (0.0, 0.0, 0.0), True
    speed = cfg.max_speed_mps * min(1.0, distance/cfg.slow_radius_m)
    scale = speed / distance
    return (scale * ex, scale * ey, scale * ez), False