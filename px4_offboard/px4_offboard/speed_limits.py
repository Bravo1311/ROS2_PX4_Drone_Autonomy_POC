"""Validation for the speed-limit ROS parameters (no ROS imports, so it is easy to test).

Both the teleop node and the mux use it in their parameter callbacks: a bad value is
rejected and the old limit stays, instead of a typo silently freezing or overspeeding the drone.
"""
import math

# Highest horizontal speed accepted: PX4's own default MPC_XY_VEL_MAX. Asking for more
# than the autopilot allows would be silently capped there, which is the confusion this avoids.
MAX_XY_VELOCITY = 12.0
MAX_Z_VELOCITY = 4.0
MAX_YAW_RATE = 3.0

# parameter name -> highest allowed value
UPPER_BOUNDS = {
    'xy_velocity_max': MAX_XY_VELOCITY,
    'max_xy_velocity': MAX_XY_VELOCITY,
    'max_z_velocity': MAX_Z_VELOCITY,
    'yaw_rate_max': MAX_YAW_RATE,
    'max_yaw_rate': MAX_YAW_RATE,
}


def check_limit(name, value):
    """Return None if ``value`` is acceptable for the limit ``name``, else a reason string."""
    upper = UPPER_BOUNDS.get(name)
    if upper is None:
        return None  # not one of our limits: not our business
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return f'{name} must be a number, got {value!r}'
    if not math.isfinite(value) or value <= 0.0:
        return f'{name} must be a positive finite number, got {value}'
    if value > upper:
        return f'{name} = {value} is above the allowed maximum {upper}'
    return None
