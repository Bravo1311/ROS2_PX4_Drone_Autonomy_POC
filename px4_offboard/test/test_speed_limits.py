import math

import pytest

from px4_offboard.speed_limits import MAX_XY_VELOCITY, check_limit


@pytest.mark.parametrize('name', ['xy_velocity_max', 'max_xy_velocity'])
def test_a_sensible_speed_is_accepted(name):
    assert check_limit(name, 4.0) is None
    assert check_limit(name, MAX_XY_VELOCITY) is None


@pytest.mark.parametrize('bad', [0.0, -1.0, math.nan, math.inf, -math.inf])
def test_zero_negative_and_non_finite_are_rejected(bad):
    assert check_limit('max_xy_velocity', bad) is not None


def test_a_speed_above_the_autopilot_maximum_is_rejected():
    assert 'maximum' in check_limit('max_xy_velocity', MAX_XY_VELOCITY + 0.1)


@pytest.mark.parametrize('bad', ['fast', None, True])
def test_non_numbers_are_rejected(bad):
    assert 'number' in check_limit('xy_velocity_max', bad)


def test_an_integer_value_is_fine():
    assert check_limit('max_z_velocity', 2) is None


def test_names_that_are_not_speed_limits_are_ignored():
    assert check_limit('controller_type', 'EVOFOX') is None
