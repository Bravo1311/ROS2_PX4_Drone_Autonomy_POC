import math

from drone_agent_core.controllers import Vec3


def world_to_body(velocity_enu: Vec3, yaw_ned_rad: float) -> Vec3:
    """East/north/up velocity to the body (forward/right/down) velocity the mux expects"""
    east, north, up = velocity_enu
    down = -up
    c, s = math.cos(yaw_ned_rad), math.sin(yaw_ned_rad)
    return (north * c + east * s, -north * s + east * c, down)