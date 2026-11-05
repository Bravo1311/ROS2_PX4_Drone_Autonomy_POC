from px4_offboard.source_select import choose_source, teleop_is_active

QUIET = dict(auto_on=False, auto_t=0.0, nav_on=False, nav_t=0.0,
             teleop_t=0.0, teleop_active_t=0.0, agent_t=0.0)
NOW = 1000.0


def pick(**changes):
    return choose_source(NOW, 5.0, **{**QUIET, **changes})


def test_nothing_fresh_is_idle():
    assert pick() == 'IDLE'


def test_zero_teleop_does_not_block_a_fresh_agent():
    assert pick(teleop_t=NOW - 0.05, agent_t=NOW - 0.1) == 'AGENT'


def test_centred_sticks_alone_are_teleop_idle_not_teleop():
    assert pick(teleop_t=NOW - 0.05) == 'TELEOP_IDLE'


def test_moving_the_sticks_beats_the_agent():
    assert pick(teleop_t=NOW, teleop_active_t=NOW - 0.1, agent_t=NOW) == 'TELEOP'


def test_teleop_stays_in_charge_for_the_holdoff_then_the_agent_may_continue():
    assert pick(teleop_active_t=NOW - 0.4, agent_t=NOW) == 'TELEOP'
    assert pick(teleop_active_t=NOW - 0.6, teleop_t=NOW, agent_t=NOW) == 'AGENT'


def test_a_silent_agent_is_dropped_after_half_a_second():
    assert pick(agent_t=NOW - 0.6) == 'IDLE'
    assert pick(agent_t=NOW - 0.6, teleop_t=NOW) == 'TELEOP_IDLE'


def test_auto_and_nav_still_outrank_everything():
    assert pick(auto_on=True, auto_t=NOW, teleop_active_t=NOW, agent_t=NOW) == 'AUTO'
    assert pick(nav_on=True, nav_t=NOW, agent_t=NOW) == 'NAV2'


def test_only_real_stick_movement_counts_as_active():
    assert not teleop_is_active(0.0, 0.0, 0.0, 0.0)
    assert not teleop_is_active(0.04, -0.04, 0.0, 0.0)
    assert teleop_is_active(0.0, 0.0, -0.2, 0.0)
