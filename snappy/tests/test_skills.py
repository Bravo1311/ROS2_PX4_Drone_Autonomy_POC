from drone_agent_core.limits import HOLD
from drone_agent_core.skills import hold_tick, takeoff_tick, goto_tick, SKILL_TICKS


def test_hold_stays_put_and_finishes_after_its_time():
    assert hold_tick((0.0, 0.0, 5.0), {"seconds": 3}, 2.9) == (HOLD, False)
    assert hold_tick((0.0, 0.0, 5.0), {"seconds": 3}, 3.0) == (HOLD, True)
    
def test_takeoff_climbs_straight_up():
    v, done = takeoff_tick((2.0, 3.0, 0.0), {"altitude_m": 5}, 0.0)
    assert v[0] == 0.0 and v[1] == 0.0 and v[2] > 0
    assert not done


def test_takeoff_is_done_at_the_target_altitude():
    v, done = takeoff_tick((2.0, 3.0, 4.9), {"altitude_m": 5}, 0.0)
    assert done and v == HOLD
    
def test_goto_flies_toward_the_target():
    v, done = goto_tick((0.0, 0.0, 5.0), {"x": 10.0, "y": 0.0, "altitude_m": 5.0}, 0.0)
    assert v[0] > 0 and v[1] == 0.0 and not done


def test_goto_is_done_at_the_target():
    v, done = goto_tick((9.9, 0.0, 5.0), {"x": 10.0, "y": 0.0, "altitude_m": 5.0}, 0.0)
    assert done and v == HOLD


def test_every_skill_in_the_table_can_be_called_the_same_way():
    assert set(SKILL_TICKS) == {"takeoff", "goto", "hold"}
    for tick in SKILL_TICKS.values():
        velocity, done = tick((0.0, 0.0, 5.0), {"altitude_m": 5.0, "x": 0.0, "y": 0.0, "seconds": 1}, 0.0)
        assert len(velocity) == 3 and isinstance(done, bool)
