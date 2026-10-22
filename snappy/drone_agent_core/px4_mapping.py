from drone_agent_core.robot import Telemetry

ARMED = 2   # PX4 VehicleStatus.arming_state: 1 = disarmed, 2 = armed

# PX4 VehicleStatus.nav_state -> short name (only the ones we expect)
NAV_STATES = {
    0: "MANUAL", 1: "ALTCTL", 2: "POSCTL", 3: "MISSION",
    4: "HOLD", 5: "RTL", 14: "OFFBOARD", 18: "TAKEOFF", 20: "LAND",
}

def to_telemetry(position, status, battery) -> Telemetry:
    remaining = battery.remaining
    return Telemetry(
        # NED: x=y, y=x, z points down
        x=position.y,
        y=position.x,
        altitude_m=-position.z,           
        battery_pct=remaining * 100 if remaining >= 0 else None,
        armed=status.arming_state == ARMED,
        mode=NAV_STATES.get(status.nav_state, f"NAV_{status.nav_state}"),
    )