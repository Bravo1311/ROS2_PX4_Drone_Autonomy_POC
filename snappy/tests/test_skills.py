import pytest

from drone_agent_core.controllers import GotoConfig
from drone_agent_core.limits import HOLD
from drone_agent_core.skills import (
    SKILL_EXPECTED_S, SKILL_TICKS, goto_expected_s, goto_tick, hold_expected_s, hold_tick,
    land_expected_s, land_tick, takeoff_expected_s, takeoff_tick,
)
from drone_agent_core.plans import SKILLS

CFG = GotoConfig()          # max speed 1.5 m/s


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
    assert set(SKILL_TICKS) == {"takeoff", "goto", "hold", "land"}
    for tick in SKILL_TICKS.values():
        velocity, done = tick((0.0, 0.0, 5.0), {"altitude_m": 5.0, "x": 0.0, "y": 0.0, "seconds": 1}, 0.0)
        assert len(velocity) == 3 and isinstance(done, bool)

def test_land_descends_straight_down_at_a_fixed_speed():
    v, done = land_tick((2.0, 3.0, 4.0), {}, 0.0)
    assert v == (0.0, 0.0, -0.5) and not done


def test_land_is_done_near_the_ground():
    assert land_tick((2.0, 3.0, 0.1), {}, 0.0) == (HOLD, True)


def test_expected_durations():
    assert takeoff_expected_s((0.0, 0.0, 0.0), {"altitude_m": 5}, CFG) == pytest.approx(5 / 1.5)
    assert goto_expected_s((0.0, 0.0, 5.0), {"x": 10.0, "y": 0.0, "altitude_m": 5.0}, CFG) == pytest.approx(10 / 1.5)
    assert hold_expected_s((0.0, 0.0, 5.0), {"seconds": 3}, CFG) == 3.0
    assert land_expected_s((0.0, 0.0, 4.15), {}, CFG) == pytest.approx(8.0)
    assert land_expected_s((0.0, 0.0, 0.0), {}, CFG) == 0.0


def test_every_skill_has_both_a_tick_and_an_expected_duration():
    assert set(SKILL_EXPECTED_S) == set(SKILL_TICKS)

def test_every_catalog_skill_has_an_implementation():
    assert set(SKILLS) == set(SKILL_TICKS)