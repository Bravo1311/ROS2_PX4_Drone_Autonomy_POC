from drone_agent_core.lastknown import LastKnown

def test_nothing_yet_is_none():
    assert LastKnown(1.0).get(0.0) is None

def test_a_recent_value_is_returned_and_an_old_one_is_not():
    known = LastKnown(1.0)
    known.update((1.0, 2.0, 3.0), 10.0)
    assert known.get(10.9) == (1.0, 2.0, 3.0)
    assert known.get(11.1) is None

def test_a_new_update_restarts_the_clock():
    known = LastKnown(1.0)
    known.update("a", 0.0)
    known.update("b", 5.0)
    assert known.get(5.5) == "b"