from types import SimpleNamespace

from drone_agent_core.px4_mapping import to_telemetry, to_pose

POS = SimpleNamespace(x=1.0, y=2.0, z=-5.0)
STATUS = SimpleNamespace(arming_state=1, nav_state=4)
BATT = SimpleNamespace(remaining=0.87)

def pos(**changes):
    base = dict(x=10.0, y=20.0, z=-5.0, heading=0.5, xy_valid=True, z_valid=True)
    return SimpleNamespace(**{**base, **changes})

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
    
def test_a_valid_position_becomes_east_north_up_and_yaw():
    assert to_pose(pos()) == ((20.0, 10.0, 5.0), 0.5)


def test_an_invalid_position_is_none():
    assert to_pose(pos(xy_valid=False)) is None
    assert to_pose(pos(z_valid=False)) is None


def test_a_non_finite_value_is_none():
    assert to_pose(pos(x=float("nan"))) is None
    assert to_pose(pos(heading=float("inf"))) is None