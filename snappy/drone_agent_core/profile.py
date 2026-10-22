from dataclasses import dataclass, fields
import yaml

class ProfileError(Exception):
    """Raised when a profile is invalid"""

AUTONOMY_MODES = ("observe", "supervised")

@dataclass(frozen=True)
class Policy:
    start_armed: bool = False
    autonomy: str = "supervised"
    approval_ttl_s: float = 60
    max_calls_per_turn: int = 10
    repeat_limit: int = 3
    max_invalid_per_turn: int = 3

    def __post_init__(self):
        if not isinstance(self.start_armed, bool):
            raise ProfileError(f"start_armed must be a boolean, got {self.start_armed!r}")
        if not isinstance(self.autonomy, str):
            raise ProfileError(f"autonomy must be a string and one of {AUTONOMY_MODES}, got {self.autonomy!r}")
        if self.autonomy not in AUTONOMY_MODES:
            raise ProfileError(f"autonomy must be one of {AUTONOMY_MODES}, got {self.autonomy!r}")
        ttl = self.approval_ttl_s
        if isinstance(ttl, bool) or not isinstance(ttl, (int, float)) or ttl <= 0:
            raise ProfileError(
                f"approval_ttl_s must be a number above 0, got {ttl!r}"
            )
        max_calls_per_turn = self.max_calls_per_turn
        if isinstance(max_calls_per_turn, bool) or not isinstance(max_calls_per_turn, (int)) or max_calls_per_turn < 1:
            raise ProfileError(
                f"max_calls_per_turn must be a number greater than 0, got {max_calls_per_turn!r}"
            )
        repeat_limit = self.repeat_limit
        if isinstance(repeat_limit, bool) or not isinstance(repeat_limit, (int)) or repeat_limit < 1:
            raise ProfileError(
                f"Repeat limit must be a number greater than 0, got {repeat_limit!r}"
            )
        invalid_limit = self.max_invalid_per_turn
        if isinstance(invalid_limit, bool) or not isinstance(invalid_limit, (int)) or invalid_limit < 1:
            raise ProfileError(
                f"max_invalid_per_turn limit must be a number greater than 0, got {invalid_limit!r}"
            )

def load_policy(data: dict) -> Policy:
    allowed = {f.name for f in fields(Policy)}
    unknown = set(data) - allowed
    if unknown:
        raise ProfileError(
            f"Unknown policy keys(s): {sorted(unknown)}; allowed: {sorted(allowed)}"
        )
    return Policy(**data)

@dataclass(frozen=True)
class Profile:
    policy: Policy

def load_profile(path) -> Profile:
    with open(path) as f:
        raw = yaml.safe_load(f) or {}
    if not isinstance(raw, dict):
        raise ProfileError("profile must be a mapping at the top level")
    unknown = set(raw) - {"policy"}
    if unknown:
        raise ProfileError(f"unknown top-level section(s): {sorted(unknown)}")
    section = raw.get("policy") or {}
    if not isinstance(section, dict):
        raise ProfileError("'policy' must be a mapping")
    return Profile(policy=load_policy(section))