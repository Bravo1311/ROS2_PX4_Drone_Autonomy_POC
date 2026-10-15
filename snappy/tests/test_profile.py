import pytest
from drone_agent_core.profile import *

@pytest.mark.parametrize(
    "data",
    [
        {"autonomy": "autonomous"},
        {"autonomy": 3},
        {"start_armed": "yes"},
        {"start_armed": 1},
        {"approval_ttl_s": 0},
        {"approval_ttl_s": -5},
        {"approval_ttl_s": "60"},
        {"approval_ttl_s": True},
    ],
)
def test_invalid_values_are_rejected(data):
    with pytest.raises(ProfileError):
        load_policy(data)

def test_empty_policy_gives_safe_defaults():
    data = {}

    policy = load_policy(data)

    assert policy.start_armed is False
    assert policy.autonomy == "supervised"
    assert policy.approval_ttl_s == 60

def test_policy_class_defaults_are_safe():
    policy = Policy()
    assert policy.start_armed is False
    assert policy.autonomy == "supervised"
    assert policy.approval_ttl_s == 60

def test_unknown_key_is_rejected():
    with pytest.raises(ProfileError):
        load_policy({"max_altitude_m": 10})

def test_load_profile_reads_yaml(tmp_path):
    path = tmp_path / "p.yaml"
    path.write_text("policy:\n autonomy: observe\n")

    profile = load_profile(path)

    assert profile.policy.autonomy == "observe"
    assert profile.policy.start_armed == False

def test_unknown_top_level_section_is_rejected(tmp_path):
    path = tmp_path / "p.yaml"
    path.write_text("limits:\n  max_alt_m: 30\n")

    with pytest.raises(ProfileError):
        load_profile(path)

