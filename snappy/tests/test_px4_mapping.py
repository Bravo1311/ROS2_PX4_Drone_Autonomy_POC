from types import SimpleNamespace

from drone_agent_core.px4_mapping import to_telemetry

POS = SimpleNamespace(x=1.0, y=2.0, z=-5.0)
STATUS = SimpleNamespace(arming_state=1, nav_state=4)
BATT = SimpleNamespace(remaining=0.87)


def test_values_are_converted_to_our_units():
    t = to_telemetry(POS, STATUS, BATT)
    assert (t.x, t.y) == (2.0, 1.0)
    assert t.altitude_m == 5.0
    assert round(t.battery_pct, 1) == 87.0
    assert t.armed is False
    assert t.mode == "HOLD"


def test_armed_and_unknown_mode():
    t = to_telemetry(POS, SimpleNamespace(arming_state=2, nav_state=99), BATT)
    assert t.armed is True
    assert t.mode == "NAV_99"


def test_invalid_battery_becomes_none():
    t = to_telemetry(POS, STATUS, SimpleNamespace(remaining=-1.0))
    assert t.battery_pct is None