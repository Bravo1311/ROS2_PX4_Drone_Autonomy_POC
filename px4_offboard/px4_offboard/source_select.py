"""Which command source is flying the drone. Pure logic, no ROS, so it can be unit-tested."""

TELEOP_THRESHOLD = 0.05   # a stick component above this counts as "the human is flying"
TELEOP_HOLDOFF_S = 0.5    # teleop stays in charge this long after the last real stick movement
AGENT_TIMEOUT_S = 0.5     # an agent that stops sending is dropped after this long


def teleop_is_active(linear_x, linear_y, linear_z, angular_z) -> bool:
    """True if the sticks are being moved (zeros at 20 Hz are not 'active')."""
    return any(abs(v) > TELEOP_THRESHOLD for v in (linear_x, linear_y, linear_z, angular_z))


def choose_source(now, inactivity_timeout, *, auto_on, auto_t, nav_on, nav_t,
                  teleop_t, teleop_active_t, agent_t) -> str:
    """Priority: AUTO > NAV2 > active TELEOP > AGENT > TELEOP_IDLE (sticks centred) > IDLE."""
    if auto_on and now - auto_t <= inactivity_timeout:
        return 'AUTO'
    if nav_on and now - nav_t <= inactivity_timeout:
        return 'NAV2'
    if now - teleop_active_t <= TELEOP_HOLDOFF_S:
        return 'TELEOP'
    if now - agent_t <= AGENT_TIMEOUT_S:
        return 'AGENT'
    if now - teleop_t <= inactivity_timeout:
        return 'TELEOP_IDLE'
    return 'IDLE'
